//! The binding tests that need a Python interpreter (`PYO3_PYTHON`):
//! the ones that read a Python result dictionary or a Python exception.
//! Moved from `router-core` in phase 8 slice B of the 2026-10 do-over;
//! every other engine test runs there without Python.

use pyo3::prelude::*;
use pyo3::types::PyDict;

use crate::*;

/// `PyRouterConfig` with every field at its documented default except
/// `search_engine`. Spelled out positionally because the binding
/// constructor's per-field defaults live in its `#[pyo3(signature=...)]`
/// and so exist only on the Python side. Returns the constructor's own
/// `PyResult`, so a test can assert the rejection of an unknown name.
fn test_router_config(search_engine: &str) -> PyResult<PyRouterConfig> {
    PyRouterConfig::new(
        2_000_000,
        10_000_000,
        30_000_000,
        false,
        true,
        false,
        false,
        false,
        false,
        false,
        None,
        None,
        None,
        search_engine.to_string(),
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        None,
        false,
        false,
        false,
        None,
        None,
        Vec::new(),
        false,
        None,
        false,
        120,
        false,
        false,
        None,
        None,
        None,
        None,
        None,
        false,
        None,
        false,
        None,
        false,
        false,
    )
}

/// The router of the auto-meander tests: a 40x40 grid at 1 um, long
/// straights of 26 cells and A* with the distance heuristic.
fn auto_meander_test_router() -> PyPhotonicRouter {
    let grid = PyGridSpec::new(40, 40, 1.0, 0.0, 0.0).unwrap();
    PyPhotonicRouter::new(
        grid,
        PyPrimitiveLibraryConfig::new(1.0, 26, 26, 1, true),
        PyAStarConfig::new(
            10000,
            1.0,
            0,
            true,
            None,
            true,
            12,
            0.35,
            3,
            true,
            0.5,
            10_000_000,
            false,
            0.0,
            0.0,
            0,
            false,
            false,
            "library".to_string(),
            "distance".to_string(),
        ),
        None,
    )
}

/// The engine names the binding accepts, and its rejection of anything
/// else (moved from `router-core`'s `grid_dijkstra` tests with the
/// binding).
#[test]
fn the_engine_name_selects_the_engine_and_rejects_unknown_names() {
    pyo3::prepare_freethreaded_python();
    assert!(test_router_config(router_core::config::SEARCH_ENGINE_ASTAR).is_ok());
    assert!(test_router_config(router_core::config::SEARCH_ENGINE_GRID_DIJKSTRA).is_ok());
    let Err(err) = test_router_config("nonsense") else {
        panic!("an unknown engine must be rejected");
    };
    let message = err.to_string();
    assert!(
        message.contains(router_core::config::SEARCH_ENGINE_ASTAR),
        "{message}"
    );
    assert!(
        message.contains(router_core::config::SEARCH_ENGINE_GRID_DIJKSTRA),
        "{message}"
    );
}

#[test]
fn auto_meander_and_commit_reserves_selected_box_cells() {
    pyo3::prepare_freethreaded_python();
    let mut router = auto_meander_test_router();
    Python::with_gil(|py| {
        let obj = router
            .route_single_net_with_auto_meander_and_commit(
                py,
                123,
                PyState::new(2, 10, 0),
                PyState::new(28, 10, 0),
                1.0,
                4.0,
                Some(0.2),
                0.0,
                2,
                20.0,
                8.0,
                1.0,
                0,
                0,
                "both",
                None,
                "fill_box_multi_bump",
            )
            .unwrap();
        let d = obj.bind(py).downcast::<PyDict>().unwrap();
        assert!(d.contains("route").unwrap());
        assert!(d.contains("polygon").unwrap());
        assert!(d.contains("selected_box").unwrap());
        assert!(d.contains("selected_grid_rect").unwrap());
        assert!(d.contains("reserved_cells").unwrap());
        assert!(d.contains("inserted_extra_length_um").unwrap());
        assert!(d.contains("bumps").unwrap());
        assert!(d.contains("side").unwrap());
    });

    let cells = router
        .inner
        .obstacle_map
        .get_net_cells(123)
        .expect("net cells should exist");
    assert!(!cells.is_empty());
    for key in cells {
        let (x, y) = router_core::obstacle_map::unpack_xy(*key);
        assert!(router.inner.obstacle_map.is_blocked(x, y));
    }
}

#[test]
fn auto_meander_commit_error_does_not_commit_route() {
    pyo3::prepare_freethreaded_python();
    let mut router = auto_meander_test_router();
    let err = Python::with_gil(|py| {
        router
            .route_single_net_with_auto_meander_and_commit(
                py,
                124,
                PyState::new(2, 10, 0),
                PyState::new(10, 10, 0),
                1.0,
                4.0,
                Some(0.2),
                0.1,
                2,
                20.0,
                1.6,
                1000.0,
                0,
                0,
                "both",
                None,
                "fill_box_multi_bump",
            )
            .unwrap_err()
    });
    assert!(!err.to_string().is_empty());
    assert!(router.inner.obstacle_map.get_net_cells(124).is_none());
}

#[test]
fn auto_meander_commit_blocks_cells_for_followup_checks() {
    pyo3::prepare_freethreaded_python();
    let mut router = auto_meander_test_router();
    let reserved_cells: Vec<(i32, i32)> = Python::with_gil(|py| {
        let obj = router
            .route_single_net_with_auto_meander_and_commit(
                py,
                125,
                PyState::new(2, 10, 0),
                PyState::new(28, 10, 0),
                1.0,
                4.0,
                Some(0.2),
                0.0,
                2,
                20.0,
                8.0,
                1.0,
                0,
                0,
                "both",
                None,
                "fill_box_multi_bump",
            )
            .unwrap();
        let d = obj.bind(py).downcast::<PyDict>().unwrap();
        d.get_item("reserved_cells")
            .unwrap()
            .unwrap()
            .extract()
            .unwrap()
    });
    assert!(!reserved_cells.is_empty());
    for (x, y) in reserved_cells {
        assert!(router.inner.obstacle_map.is_blocked(x, y));
    }
}
