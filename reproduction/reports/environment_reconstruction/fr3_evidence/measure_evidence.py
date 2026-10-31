"""Read-only measurements of source evidence; no planner results are inputs."""
from pathlib import Path
import hashlib
import json
import tarfile
import numpy as np
from PIL import Image
from scipy.ndimage import label, find_objects

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
image_path = OUT / 'figure6_actual-000.png'
im = np.array(Image.open(image_path))[:, :, :3]
mask = (im.max(2) < 65) & (im.max(2) - im.min(2) < 10)
labels, _ = label(mask)
components = []
for i, region in enumerate(find_objects(labels), 1):
    if region is None:
        continue
    area = int((labels[region] == i).sum())
    y, x = region
    if area > 150 and y.stop < 650:
        components.append(dict(area_pixels=area,
                               bbox_xyxy=[x.start, y.start, x.stop, y.stop]))
# Manual plane landmarks; rear corner is partially occluded and least certain.
# Coordinates are visually read from the original image, not fitted to metrics.
plane_unit_square = np.array([[0, 0], [0, 1], [1, 1], [1, 0]], float)
plane_image = np.array([[720, 431], [1049, 520], [490, 880], [185, 600]], float)
A, b = [], []
for (x, y), (u, v) in zip(plane_unit_square, plane_image):
    A.extend([[x, y, 1, 0, 0, 0, -u*x, -u*y],
              [0, 0, 0, x, y, 1, -v*x, -v*y]])
    b.extend([u, v])
H = np.r_[np.linalg.solve(A, b), 1].reshape(3, 3)
base_pixels = [[574, 520], [570, 510], [557, 520], [600, 495]]
base_plane = []
for pixel in base_pixels:
    q = np.linalg.inv(H) @ np.r_[pixel, 1]
    base_plane.append((q[:2] / q[2]).tolist())
obj = ROOT / 'csdecomp/tests/test_assets/table.obj'
vertices = np.array([[float(t) for t in line.split()[1:]]
                     for line in obj.read_text().splitlines() if line.startswith('v ')])
result = dict(
    source_image=dict(path=str(image_path.relative_to(ROOT)),
                      dimensions=list(Image.open(image_path).size), sha256=sha(image_path),
                      pdf_page_one_based=8, pdf_image_object=553),
    threshold_components=dict(method='RGB maximum < 65, RGB spread < 10, area > 150, bottom y < 650',
                              warning='not object identities; some overlap, translucency and occlusion merge/split spheres',
                              count=len(components), components=components),
    manually_observed_table_clipped_sphere_regions_xyxy=[[698,486,725,504],[801,484,829,506],[870,469,899,498]],
    plane_measurement=dict(landmarks_pixels=plane_image.tolist(),
                           plane_unit_square=plane_unit_square.tolist(),
                           base_candidates_pixels=base_pixels,
                           base_candidates_plane=base_plane,
                           warning='illustrative 2D homography only; no camera calibration or 3D recovery; no confidence interval'),
    public_visual_table=dict(path=str(obj.relative_to(ROOT)), sha256=sha(obj),
                             vertex_count=len(vertices), minimum=vertices.min(0).tolist(), maximum=vertices.max(0).tolist(),
                             first_commit='eb699f7c3070e4510d4eb17056e47878ea985c9d',
                             first_commit_date='2025-02-17T12:21:16-05:00'),
    public_collision_table=dict(path='csdecomp/tests/test_assets/table.urdf',
                                sha256=sha(ROOT/'csdecomp/tests/test_assets/table.urdf'),
                                box_size=[2.5,2.5,.2], local_center=[0,0,-.1],
                                note='public test asset, not recovered author FR3 directive'),
    no_inputs_from_planner_results=True,
)
(OUT/'measurements.json').write_text(json.dumps(result, indent=2)+'\n')
archive = ROOT/'reproduction/reports/paper_completion/source_evidence/arxiv_2504.10783v1.tar.gz'
member='figures/franka_benchmark_gradient.pdf'
with tarfile.open(archive) as tf:
    content=tf.extractfile(member).read()
(OUT/'author_figure6.pdf').write_bytes(content)
sources = dict(
    paper=dict(url='https://arxiv.org/pdf/2504.10783v1', path='reproduction/paper/2504.10783v1.pdf',sha256=sha(ROOT/'reproduction/paper/2504.10783v1.pdf')),
    arxiv_source=dict(url='https://arxiv.org/src/2504.10783v1',path=str(archive.relative_to(ROOT)),sha256=sha(archive)),
    author_figure=dict(member=member,sha256=sha(OUT/'author_figure6.pdf')),
    webpage=dict(url='https://sites.google.com/view/GPUPolytopes',access='success via web tool 2026-10-26',
                 observation='Author webpage explicitly describes linked video as the hardware setup; no FR3 scene parameters exposed.'),
    video=dict(url='https://www.youtube.com/watch?v=BXf0h8ePw0M',access='web tool returned Online fetch throttled on embed and watch URLs',
               observation='No video content used; no inferred frames or measurements.'),
    public_table_mesh=dict(url='https://github.com/wernerpe/csdecomp/blob/eb699f7c3070e4510d4eb17056e47878ea985c9d/csdecomp/tests/test_assets/table.obj'),
    public_table_urdf=dict(url='https://github.com/wernerpe/csdecomp/blob/eb699f7c3070e4510d4eb17056e47878ea985c9d/csdecomp/tests/test_assets/table.urdf'),
    public_kinova_directive=dict(path='csdecomp/tests/test_assets/directives/kinova_sens_on_table.yml',table_translation=[.4,0,0],
                                 note='Related public example only; not evidence of exact FR3 transform'),
)
(OUT/'sources.json').write_text(json.dumps(sources,indent=2)+'\n')
print(json.dumps(dict(image_dimensions=result['source_image']['dimensions'], threshold_components=len(components),
                     table_visual_bounds=[vertices.min(0).tolist(), vertices.max(0).tolist()],
                     base_plane=base_plane), indent=2))
