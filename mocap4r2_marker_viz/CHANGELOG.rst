^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mocap4r2_marker_viz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-11-14)
------------------
* Rewritten: markers as sphere lists (labeled, unlabeled, occluded), rigid bodies as axes with names, skeletons as bones; DELETEALL and stable ids (no flicker, no unbounded ids); throttled to max_rate; cleared when data goes stale.
* No axis conversions: drivers publish Z-up.
* Removed mocap4r2_marker_viz_srvs and mocap4r2_marker_publisher.
* Contributors: Francisco Martín Rico, José Miguel Guerrero, Juan Carlos Manzanares Serrano, Manuel Fernández Carmona
