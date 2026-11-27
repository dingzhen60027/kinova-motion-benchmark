"""Shared, inspectable composition for the MuJoCo desktop and exported video.

Endpoint panels are actual MuJoCo renders of the original task poses. The
central view always uses saved simulated states. Styling changes only colors.
"""
from functools import lru_cache
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

from .guidance import START, GOAL


INK = (35, 46, 59)
MUTED = (96, 109, 124)
PAPER = (249, 250, 252)
RULE = (222, 227, 234)
BLUE = (42, 96, 196)
GREEN = (20, 137, 97)


@lru_cache(maxsize=32)
def font(size):
    for name in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 'DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def presentation_camera(mujoco, guide):
    points = np.vstack((guide.trace, guide.start_pose, guide.goal_pose))
    low, high = points.min(axis=0), points.max(axis=0)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (low + high) / 2
    camera.distance = max(2.15, 1.9 * float(np.max(high-low)))
    camera.azimuth, camera.elevation = 145., -20.
    return camera


class Presentation:
    def __init__(self, mujoco, model, guide, camera):
        self.mj, self.model, self.guide, self.camera = mujoco, model, guide, camera
        self._cache = {}
        self._endpoint_master = None
        # Preserve every collision/contact field. These are transient display colors.
        for i in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
            if name.startswith('obstacle_visual_'):
                model.geom_rgba[i] = [.76, .43, .19, 1.]
            elif name.startswith('fixed_collision_visual_'):
                model.geom_rgba[i] = [.61, .67, .73, .38]
        self.robot_visuals = []
        for i in range(model.ngeom):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
            body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[i])) or ''
            if name.startswith('visual_') and ('_kinova_' in body or '_robotiq_' in body):
                self.robot_visuals.append(i)

    @staticmethod
    def layout(width, height):
        sidebar = max(170, min(310, int(width * .24)))
        header, footer = 78, 108
        return width-sidebar, max(120, height-header-footer), sidebar, header, footer

    def thumbnails(self, width, height):
        key = (width, height)
        if key in self._cache:
            return self._cache[key]
        if self._endpoint_master is not None:
            frames = [ImageOps.pad(frame, (width, height), color=PAPER)
                      for frame in self._endpoint_master]
            self._cache[key] = frames
            return frames
        mj, model = self.mj, self.model
        model.vis.global_.offwidth = max(width, model.vis.global_.offwidth)
        model.vis.global_.offheight = max(height, model.vis.global_.offheight)
        option = mj.MjvOption()
        option.geomgroup[:] = [1, 1, 1, 0, 0, 0]
        data = mj.MjData(model)
        saved_colors = model.geom_rgba.copy()
        frames = []
        try:
            with mj.Renderer(model, height=height, width=width) as renderer:
                for qpos, color in zip(self.guide.endpoint_qpos, (START, GOAL)):
                    model.geom_rgba[:] = saved_colors
                    model.geom_rgba[self.robot_visuals] = [*color[:3], 1.]
                    data.qpos[:] = qpos
                    mj.mj_forward(model, data)
                    renderer.update_scene(data, camera=self.camera, scene_option=option)
                    self.screen_right = np.cross(renderer.scene.camera[0].forward,
                                                 renderer.scene.camera[0].up)
                    frames.append(Image.fromarray(renderer.render()))
        finally:
            model.geom_rgba[:] = saved_colors
        self._cache[key] = frames
        self._endpoint_master = frames
        return frames

    def panels(self, width, height, timestamp, paused=False, speed=1.):
        main_w, main_h, side_w, top_h, bottom_h = self.layout(width, height)
        top = Image.new('RGB', (width, top_h), PAPER)
        draw = ImageDraw.Draw(top)
        category = {'dense_clutter':'密集障碍', 'sparse_clutter':'稀疏障碍',
                    'bottleneck':'瓶颈布局', 'route_choice':'多路线布局', 'open_control':'基础环境'}
        title = category.get(self.guide.case.get('family'), '机械臂运动规划')
        method = {'EI_ZO':'EI-ZO + SCSTO'}.get(self.guide.result['method'], self.guide.result['method'])
        draw.text((22, 8), 'KINOVA GEN3  /  MOTION PLANNING', font=font(12), fill=MUTED)
        draw.text((22, 27), title + ' · 轨迹执行', font=font(23), fill=INK)
        if width >= 900:
            draw.text((width-350, 13), method, font=font(19), fill=INK)
            draw.text((width-350, 42), self.guide.case['problem_id'], font=font(12), fill=MUTED)
        draw.line((0, top_h-1, width, top_h-1), fill=RULE)

        side = Image.new('RGB', (side_w, main_h), PAPER)
        draw = ImageDraw.Draw(side)
        draw.line((0, 0, 0, main_h), fill=RULE)
        cell = main_h // 2
        thumb_w, thumb_h = side_w-24, max(60, cell-79)
        pictures = self.thumbnails(thumb_w, thumb_h)
        for i, (label, sub, color, picture) in enumerate(zip(
            ('S  出发姿态', 'G  目标姿态'),
            ('原始 q_start', '原始 q_goal'), (BLUE, GREEN), pictures)):
            y = i * cell
            draw.rectangle((13, y+16, 16, y+44), fill=color)
            draw.text((25, y+10), label, font=font(18), fill=color)
            draw.text((25, y+39), sub, font=font(12), fill=MUTED)
            side.paste(picture, (12, y+65))
            if i == 0:
                draw.line((12, cell-1, side_w-12, cell-1), fill=RULE)

        bottom = Image.new('RGB', (width, bottom_h), PAPER)
        draw = ImageDraw.Draw(bottom)
        draw.line((0, 0, width, 0), fill=RULE)
        duration = self.guide.end_time-self.guide.start_time
        elapsed = float(np.clip(timestamp-self.guide.start_time, 0, duration))
        fraction = elapsed/duration if duration else 1.
        state = '播放结束' if fraction >= 1 else '已暂停' if paused else '正在播放'
        draw.text((22, 10), f'{state}    {elapsed:05.2f} / {duration:.2f} s', font=font(16), fill=INK)
        if width >= 900 and 'reference_duration_s' in self.guide.result:
            reference = float(self.guide.result['reference_duration_s'])
            draw.text((350, 13), f'轨迹 {reference:.2f} s  +  终点保持 {max(0., duration-reference):.2f} s',
                      font=font(12), fill=MUTED)
        right = f'{fraction:.0%}   ·   {speed:g}×'
        draw.text((width-145, 10), right, font=font(16), fill=INK)
        draw.rectangle((22, 41, width-22, 45), fill=RULE)
        if fraction:
            draw.rectangle((22, 41, 22+(width-44)*fraction, 45), fill=BLUE)
        failed = [k for k,v in self.guide.result.get('checks',{}).items() if v is False]
        if failed == ['acceleration_limits']:
            maxima = self.guide.result.get('maximum_acceleration_rad_s2', [])
            value = f'{max(maxima):.4f}' if maxima else '—'
            note = f'整段执行结果：加速度超限（峰值 {value} rad/s²）；无碰撞、到达终点' if all(
                self.guide.result.get('checks',{}).get(k) for k in ('contact_free','independent_collision_free','endpoint')) else f'执行检查未通过：加速度峰值 {value} rad/s²'
        elif failed:
            note = '执行检查未通过：' + ', '.join(failed)
        else:
            note = ('整段执行结果：数值检查通过' if self.guide.result['status']=='execution_success_numerical'
                    else self.guide.result['status'])
        draw.text((22, 51), note, font=font(13), fill=(150,90,26) if failed else GREEN)
        controls = '空格 播放/暂停   ·   Home 重播   ·   1 / 2 首帧/末帧   ·   鼠标拖动 旋转视角'
        draw.text((22, 78), controls, font=font(12), fill=MUTED)
        return top, side, bottom

    def compose(self, main_frame, width, height, timestamp, paused=False, speed=1.):
        main_w, main_h, side_w, header, footer = self.layout(width, height)
        top, side, bottom = self.panels(width, height, timestamp, paused, speed)
        image = Image.new('RGB', (width, height), PAPER)
        image.paste(Image.fromarray(main_frame), (0, header))
        image.paste(top, (0, 0))
        image.paste(side, (main_w, header))
        image.paste(bottom, (0, height-footer))
        return np.asarray(image)
