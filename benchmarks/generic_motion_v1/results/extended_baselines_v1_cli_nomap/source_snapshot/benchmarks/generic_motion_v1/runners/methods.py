"""Declared method capabilities; objective support is never inferred from output."""

INDEPENDENT = frozenset({'RRTCONNECT', 'RRTSTAR'})
LOCAL_EXTENSIONS = frozenset({'CHOMP', 'STOMP', 'TRAJOPT'})
EXTENSIONS = INDEPENDENT | LOCAL_EXTENSIONS
METHODS = ('EI_ZO', 'LGCS', 'NTO_PAPER', 'NTO_REFINED', 'NTO',
           'RRTCONNECT', 'RRTSTAR', 'CHOMP', 'STOMP', 'TRAJOPT')


def objectives(method):
    if method not in METHODS:
        raise ValueError(f'Unknown method: {method}')
    return ('minimum_distance',) if method in EXTENSIONS | {'LGCS'} else ('minimum_distance', 'minimum_time')


def catalog():
    return [dict(method=m, objectives=list(objectives(m)),
                 initialization='fixed endpoints; independent native sampling' if m in INDEPENDENT
                 else 'independently validated native DRM path',
                 roadmap_required=m not in INDEPENDENT,
                 quality_metric='validated Euclidean PWL length in rad' if m in EXTENSIONS else 'objective dependent',
                 native_objective=('smoothness plus collision penalty' if m in LOCAL_EXTENSIONS
                                   else 'first feasible path then simplification' if m == 'RRTCONNECT'
                                   else 'path length' if m == 'RRTSTAR' else 'see frozen method profile'))
            for m in METHODS]
