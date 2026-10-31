"""CPU-only exact outer extent of finite voxel-ball covers of source spheres."""
import argparse
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
BASE = HERE.parents[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=HERE / 'sphere_cover_extent.json')
    args = parser.parse_args()
    rows = []
    for path in (BASE / 'scenes').glob('*.json'):
        scene = json.loads(path.read_text())
        xyz = np.asarray(scene['voxels_xyz']).reshape(-1, 3)
        for obstacle, cover in zip(scene['obstacle_geometry'], scene['voxelization']['obstacles']):
            if obstacle['shape'] != 'sphere':
                continue
            begin = cover['first_voxel']
            sub = xyz[begin:begin + cover['voxel_count']]
            reach = float(np.max(np.linalg.norm(sub - np.array(obstacle['center_m']), axis=1)) + scene['voxel_radius_m'])
            rows.append(dict(scene_id=scene['scene_id'], split=scene['split'], family=scene['family'],
                obstacle_id=obstacle['obstacle_id'], source_radius_m=obstacle['radius_m'],
                cover_outer_radius_m=reach, outward_excess_m=reach-obstacle['radius_m'],
                radius_ratio=reach/obstacle['radius_m']))
    if args.output.exists():
        assert json.loads(args.output.read_text()) == rows, 'Existing evidence differs; refusing overwrite'
    else:
        args.output.write_text(json.dumps(rows, indent=2) + '\n')
    print(json.dumps(dict(sphere_count=len(rows),
        outward_excess_m=[min(r['outward_excess_m'] for r in rows), max(r['outward_excess_m'] for r in rows)],
        outer_radius_ratio=[min(r['radius_ratio'] for r in rows), max(r['radius_ratio'] for r in rows)],
        meaning='Maximum radial extent of the union; not an isotropic inflated sphere and not a volume ratio.'), indent=2))


if __name__ == '__main__':
    main()
