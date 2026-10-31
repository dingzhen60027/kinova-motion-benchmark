"""Declared method capabilities; objective support is never inferred from output."""

INDEPENDENT = frozenset({'RRTCONNECT', 'RRTSTAR'})
LOCAL_EXTENSIONS = frozenset({'CHOMP', 'STOMP', 'TRAJOPT'})
EXTENSIONS = INDEPENDENT | LOCAL_EXTENSIONS
METHODS = ('EI_ZO', 'LGCS', 'NTO_PAPER', 'NTO_REFINED', 'NTO',
           'RRTCONNECT', 'RRTSTAR', 'CHOMP', 'STOMP', 'TRAJOPT')

# Stable stored IDs remain compatible with historical run evidence. Display
# names describe implementation scope rather than silently renaming results.
IMPLEMENTATIONS = {
    'EI_ZO': dict(display_name='DRM + EI-ZO + LSCS / SCSTO',
        implementation='native CSDecomp and author SCSPlanning; Clarabel solver adaptation',
        distinction='Collision feedback, native region editing and corridor repair retained'),
    'LGCS': dict(display_name='DRM + EI-ZO + LGCS reconstruction',
        implementation='local perspective GCS MISOCP transcription; explicit Gurobi or SCIP backend',
        distinction='Global optimum concerns the supplied local region graph, not full free C-space'),
    'NTO_PAPER': dict(display_name='NTO published-profile reconstruction',
        implementation='Drake KTO; SNOPT path and IPOPT time; published spline orders',
        distinction='Author NTO transcription unavailable; 149 interior collision bindings and initialization declared'),
    'NTO_REFINED': dict(display_name='NTO collision-refined adaptation',
        implementation='NTO reconstruction plus positive clearance and iterative collision collocation',
        distinction='Same frozen geometry and objectives; local numerical strengthening disclosed'),
    'NTO': dict(display_name='NTO legacy adaptation',
        implementation='local Drake KTO/IPOPT transcription with resampled initialization',
        distinction='Separate historical method; not an alias for NTO_PAPER or NTO_REFINED'),
    'RRTCONNECT': dict(display_name='OMPL RRTConnect + vertex reduction',
        implementation='official OMPL 2.0.1 native tree search and path simplification',
        distinction='Fixed seven-dimensional bounded state space; canonical collision callbacks'),
    'RRTSTAR': dict(display_name='OMPL RRT* + vertex reduction',
        implementation='official OMPL 2.0.1 native rewiring search and path simplification',
        distinction='Validated native incumbent logging is an explicit configuration opt-in'),
    'CHOMP': dict(display_name='CHOMP adaptation (C-space functional)',
        implementation='local acceleration-metric covariant descent with exact discrete functional gradient',
        distinction='Canonical minimum-clearance potential integrated over C-space arc length; differs from original workspace body-point functional'),
    'STOMP': dict(display_name='STOMP (official core, canonical task)',
        implementation='unmodified ros-industrial STOMP native core with local correlated-noise and geometry task callbacks',
        distinction='Native smoothness plus waypoint collision potential, not pure path-length optimization'),
    'TRAJOPT': dict(display_name='TrajOpt adaptation (sampled collision)',
        implementation='author BasicTrustRegionSQP and BPMPD core with canonical signed-distance callbacks',
        distinction='Segment collocation replaces OpenRAVE swept-volume constraints; squared-difference objective differs from evaluated path length'),
}


def objectives(method):
    if method not in METHODS:
        raise ValueError(f'Unknown method: {method}')
    return ('minimum_distance',) if method in EXTENSIONS | {'LGCS'} else ('minimum_distance', 'minimum_time')


def catalog():
    return [dict(method=m, **IMPLEMENTATIONS[m], objectives=list(objectives(m)),
                 initialization='fixed endpoints; independent native sampling' if m in INDEPENDENT
                 else 'independently validated native DRM path',
                 roadmap_required=m not in INDEPENDENT,
                 quality_metric='validated Euclidean PWL length in rad' if m in EXTENSIONS else 'objective dependent',
                 native_objective=('smoothness plus collision penalty' if m in LOCAL_EXTENSIONS
                                   else 'first feasible path then simplification' if m == 'RRTCONNECT'
                                   else 'path length' if m == 'RRTSTAR' else 'see frozen method profile'))
            for m in METHODS]
