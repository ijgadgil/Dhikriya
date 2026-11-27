#!/usr/bin/env python3
"""Generate Gazebo Harmonic worlds per scenario from town.yaml + scenarios/*.yaml, for two maps.

Roads, lane markings, the junction, road signs (text rendered to PBR textures), barrels,
barriers and the scenario actors are all produced from the same YAML the harness uses, so the
simulated world and the evaluation ground truth cannot drift apart.

Two worlds per scenario share one road graph (town.yaml), so ground truth is identical:
  worlds/<scenario>.sdf            minimal synthetic town (controlled baseline)
  worlds/<scenario>_mcity_vla.sdf  earlier Mcity map (model city_vla) under the same graph (config/mcity_vla.yaml),
                                   with solid buildings and Indian-street clutter (config/street_indian.yaml)

  python3 gen_worlds.py [--mirror-signs] [--offline]
--mirror-signs  flip sign textures if text appears mirrored in your Gazebo build
--offline       replace Fuel models (bull, walking person) with simple shapes
"""
import argparse
import glob
import math
import os
import sys

import yaml
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from cogndn_sim import street as streetlib  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
CFG = os.path.join(ROOT, 'config')
MODELS = os.path.join(ROOT, 'models')
WORLDS = os.path.join(ROOT, 'worlds')
FUEL = {'bull': 'https://fuel.gazebosim.org/1.0/GoogleResearch/models/Schleich_Hereford_Bull',
        'person': 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Walking person'}


def font(size):
    for f in ('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
              '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf'):
        if os.path.exists(f):
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()


def sign_texture(path, lines, mirror=False, color=(255, 200, 0)):
    W, H = 1024, 512
    im = Image.new('RGB', (W, H), color)
    d = ImageDraw.Draw(im)
    d.rectangle([12, 12, W - 13, H - 13], outline=(0, 0, 0), width=18)
    f = font(118 if len(lines) <= 3 else 92)
    hs = [d.textbbox((0, 0), ln, font=f) for ln in lines]
    total = sum(b[3] - b[1] for b in hs) + 30 * (len(lines) - 1)
    y = (H - total) / 2
    for ln, b in zip(lines, hs):
        w = b[2] - b[0]
        d.text(((W - w) / 2 - b[0], y - b[1]), ln, fill=(0, 0, 0), font=f)
        y += (b[3] - b[1]) + 30
    if mirror:
        im = im.transpose(Image.FLIP_LEFT_RIGHT)
    im.save(path)


def sign_model(sid, lines, mirror):
    d = os.path.join(MODELS, sid)
    os.makedirs(os.path.join(d, 'materials', 'textures'), exist_ok=True)
    color = (255, 140, 0) if any('WORK' in ln or 'CLOSED' in ln for ln in lines) else (255, 205, 0)
    sign_texture(os.path.join(d, 'materials', 'textures', 'face.png'), lines, mirror, color)
    open(os.path.join(d, 'model.config'), 'w').write(
        '<?xml version="1.0"?><model><name>%s</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>'
        '<description>Road sign</description></model>' % sid)
    open(os.path.join(d, 'model.sdf'), 'w').write('''<?xml version="1.0"?>
<sdf version="1.9"><model name="{sid}"><static>true</static>
  <link name="pole"><pose>0 0 1.0 0 0 0</pose>
    <visual name="v"><geometry><cylinder><radius>0.06</radius><length>2.0</length></cylinder></geometry>
      <material><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
    <collision name="c"><geometry><cylinder><radius>0.06</radius><length>2.0</length></cylinder></geometry></collision></link>
  <link name="board"><pose>-0.05 0 2.6 0 0 0</pose>
    <visual name="face"><geometry><box><size>0.04 2.6 1.3</size></box></geometry>
      <material><diffuse>1 1 1 1</diffuse><ambient>1 1 1 1</ambient>
        <pbr><metal><albedo_map>model://{sid}/materials/textures/face.png</albedo_map><metalness>0</metalness>
        <roughness>0.8</roughness></metal></pbr></material></visual></link>
</model></sdf>
'''.format(sid=sid))


def box(name, x, y, z, sx, sy, sz, yaw, rgb, collide=False):
    col = ('<collision name="c"><geometry><box><size>%g %g %g</size></box></geometry></collision>' % (sx, sy, sz)
           if collide else '')
    return ('<model name="%s"><static>true</static><pose>%g %g %g 0 0 %g</pose><link name="l">'
            '<visual name="v"><geometry><box><size>%g %g %g</size></box></geometry><material><diffuse>%s 1</diffuse>'
            '<ambient>%s 1</ambient></material></visual>%s</link></model>\n'
            % (name, x, y, z, yaw, sx, sy, sz, rgb, rgb, col))


def roads(town):
    out, lw = [], town['lane_width']
    for r in town['roads']:
        (x0, y0), (x1, y1) = r['start'], r['end']
        L, yaw = math.hypot(x1 - x0, y1 - y0), math.atan2(y1 - y0, x1 - x0)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        out.append(box('road_' + r['id'], cx, cy, 0.01, L, 2 * lw + 1.0, 0.02, yaw, '0.18 0.18 0.19'))
        n = int(L // 6)
        for i in range(n):        # dashed centre line
            t = (i + 0.5) / n
            out.append(box('dash_%s_%d' % (r['id'], i), x0 + t * (x1 - x0), y0 + t * (y1 - y0), 0.025, 3.0, 0.15, 0.01,
                           yaw, '0.95 0.95 0.95'))
        for side in (-1, 1):      # edge lines
            ox, oy = -math.sin(yaw) * side * lw, math.cos(yaw) * side * lw
            out.append(box('edge_%s_%d' % (r['id'], side), cx + ox, cy + oy, 0.025, L, 0.12, 0.01, yaw,
                           '0.95 0.95 0.95'))
    for it in town.get('intersections', []):
        c, s = it['center'], it['size']
        out.append(box('junction_' + it['id'], c[0], c[1], 0.012, s, s, 0.02, 0.0, '0.2 0.2 0.21'))
    return ''.join(out)


def objects(sc, offline, mc=None):
    out = []
    for s in sc.get('signs', []):
        x, y, _ = s['pose']
        if mc:                    # move roadside signs out to the wider Mcity kerb
            y += -mc['sign_shift_m'] if y < 0 else mc['sign_shift_m']
        out.append('<include><name>%s</name><uri>model://%s_%s</uri><pose>%g %g 0 0 0 %g</pose></include>\n'
                   % (s['id'], sc['id'], s['id'], x, y, math.radians(s.get('facing_deg', 180)) - math.pi))
    for o in sc.get('objects', []):
        k = o['kind']
        if k == 'barrels':
            (x0, y0), (x1, y1) = o['from'], o['to']
            n = int(math.hypot(x1 - x0, y1 - y0) // o.get('spacing', 4.0)) + 1
            for i in range(n):
                t = i / max(1, n - 1)
                out.append('<model name="barrel_%d"><static>true</static><pose>%g %g 0.5 0 0 0</pose><link name="l">'
                           '<visual name="v"><geometry><cylinder><radius>0.3</radius><length>1.0</length></cylinder>'
                           '</geometry><material><diffuse>1 0.45 0 1</diffuse><ambient>1 0.45 0 1</ambient></material>'
                           '</visual><visual name="band"><pose>0 0 0.15 0 0 0</pose><geometry><cylinder><radius>0.31'
                           '</radius><length>0.15</length></cylinder></geometry><material><diffuse>1 1 1 1</diffuse>'
                           '</material></visual><collision name="c"><geometry><cylinder><radius>0.3</radius><length>1.0'
                           '</length></cylinder></geometry></collision></link></model>\n'
                           % (i, x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
        elif k == 'barrier':
            x, y, yaw = o['pose']
            w = mc['barrier_width_m'] if mc else 7.5
            out.append(box('barrier', x, y, 0.6, 0.3, w, 1.2, yaw, '0.85 0.1 0.1', collide=True))
        elif k in FUEL:
            x, y, yaw = o['pose']
            if offline:
                sz = (1.8, 0.6, 1.4) if k == 'bull' else (0.4, 0.4, 1.75)
                out.append(box(o['id'], x, y, sz[2] / 2, sz[0], sz[1], sz[2], yaw,
                               '0.35 0.2 0.1' if k == 'bull' else '0.2 0.3 0.8', collide=True))
            else:
                uri = 'model://cow' if k == 'bull' else FUEL[k]     # the Fuel bull is a toy-sized scan
                out.append('<include><name>%s</name><uri>%s</uri><pose>%g %g 0 0 0 %g</pose></include>\n'
                           % (o['id'], uri, x, y, yaw))
    return ''.join(out)


WORLD = '''<?xml version="1.0"?>
<!-- generated by gen_worlds.py from {sid}.yaml - do not edit by hand -->
<sdf version="1.9"><world name="{wname}">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.8 0.95 1</background><shadows>true</shadows></scene>
  <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 50 0 0 0</pose>
    <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.2 -0.9</direction></light>
  <model name="ground"><static>true</static><link name="l">
    <collision name="c"><geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry></collision>
    <visual name="v"><geometry><plane><normal>0 0 1</normal><size>1000 1000</size></plane></geometry>
      <pose>0 0 {gz} 0 0 0</pose>
      <material><diffuse>0.35 0.55 0.25 1</diffuse><ambient>0.35 0.55 0.25 1</ambient></material></visual></link></model>
{roads}
{objects}
  <include><name>prius_vla</name><uri>model://prius_vla</uri><pose>{x} {y} 0.05 0 0 {yaw}</pose></include>
</world></sdf>
'''


def mesh_to_town(mc):
    """Return f(x, y) mapping city_vla mesh units to town metres (same transform as mcity_include)."""
    sc, (jx, jy), yaw = mc['scale'], mc['junction_mesh'], math.radians(mc['yaw_deg'])
    c, s = math.cos(yaw), math.sin(yaw)
    tx, ty = 105.0 - (c * sc * jx - s * sc * jy), 0.0 - (s * sc * jx + c * sc * jy)
    return lambda x, y: (tx + sc * (c * x - s * y), ty + sc * (s * x + c * y))


def buildings(mc):
    """Solid blocks behind the city_vla facades so the buildings are no longer hollow."""
    f, sc, out = mesh_to_town(mc), mc['scale'], []
    for i, (x0, y0, x1, y1, h) in enumerate(mc.get('buildings', [])):
        cx, cy = f((x0 + x1) / 2, (y0 + y1) / 2)
        out.append(box('building_block_%d' % i, cx, cy, sc * h / 2, sc * (x1 - x0), sc * (y1 - y0), sc * h,
                       math.radians(mc['yaw_deg']), '0.36 0.33 0.31'))
    return ''.join(out)


def _include(name, uri, x, y, yaw, static=True, speed=None, z=0.05):
    plug = ''
    if speed:
        plug = ('<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">'
                '<initial_linear>%g 0 0</initial_linear></plugin>' % speed)
    return ('<include><name>%s</name><uri>%s</uri><static>%s</static><pose>%g %g %g 0 0 %g</pose>%s</include>\n'
            % (name, uri, 'true' if static and not speed else 'false', x, y, z, yaw, plug))


def _walker(p, dae):
    """Animated pedestrian (visible to camera and LiDAR) walking its looped script."""
    k = p.get('scale', 1.0)
    w = ''.join('<waypoint><time>%.2f</time><pose>%g %g %g 0 0 %.4f</pose></waypoint>' % (t, x, y, 1.0 * k, yaw)
                for t, x, y, yaw in streetlib.waypoints(p))
    return ('<actor name="%s"><skin><filename>%s</filename><scale>%g</scale></skin>'
            '<animation name="walking"><filename>%s</filename><interpolate_x>true</interpolate_x></animation>'
            '<script><loop>true</loop><delay_start>0</delay_start><auto_start>true</auto_start>'
            '<trajectory id="0" type="walking">%s</trajectory></script></actor>\n' % (p['id'], dae, k, dae, w))


def _ped_body(p):
    """Invisible physics body that street_bodies_node keeps inside the actor (actors have no collision)."""
    k = p.get('scale', 1.0)
    x, y = p['path'][0]
    r, L = 0.25 * k, 1.2 * k                    # capsule: 1.7 m tall adult, about 1 m child
    return ('<model name="%s_body"><pose>%g %g 0.02 0 0 0</pose><link name="body">'
            '<inertial><pose>0 0 0.1 0 0 0</pose><mass>%g</mass><inertia><ixx>1e4</ixx><iyy>1e4</iyy><izz>1</izz>'
            '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            '<collision name="c"><pose>0 0 %g 0 0 0</pose><geometry><capsule><radius>%g</radius><length>%g</length>'
            '</capsule></geometry><surface><friction><ode><mu>0.01</mu><mu2>0.01</mu2></ode></friction>'
            '<contact><collide_without_contact>true</collide_without_contact></contact></surface>'
            '</collision></link>'
            '<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">'
            '<topic>/model/%s_body/cmd_vel</topic></plugin>'
            '<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">'
            '<dimensions>2</dimensions><odom_publish_frequency>30</odom_publish_frequency>'
            '<odom_topic>/model/%s_body/odometry</odom_topic></plugin></model>\n'
            % (p['id'], x, y, 70 * k, r + L / 2, r, L, p['id'], p['id']))


def bridge_mcity(st):
    """bridge.yaml with the mcity_vla world name plus the topics street_bodies_node uses (pedestrian bodies,
    traffic speed commands)."""
    base = open(os.path.join(CFG, 'bridge.yaml')).read()
    out = base.replace('/world/cogndn_town/', '/world/mcity_vla_world/').replace(
        'generated cogndn_town worlds', 'mcity_vla worlds (generated by gen_worlds.py)')
    for v in st.get('traffic', []):       # oncoming vehicles: street_bodies_node makes them yield to the car
        out += ('- ros_topic_name: /street/%s/cmd_vel\n  gz_topic_name: /model/%s/cmd_vel\n'
                '  ros_type_name: geometry_msgs/msg/Twist\n  gz_type_name: gz.msgs.Twist\n  direction: ROS_TO_GZ\n'
                % (v['id'], v['id']))
    for p in st.get('pedestrians', []):
        i = p['id']
        out += ('- ros_topic_name: /street/%s/cmd_vel\n  gz_topic_name: /model/%s_body/cmd_vel\n'
                '  ros_type_name: geometry_msgs/msg/Twist\n  gz_type_name: gz.msgs.Twist\n  direction: ROS_TO_GZ\n'
                '- ros_topic_name: /street/%s/odom\n  gz_topic_name: /model/%s_body/odometry\n'
                '  ros_type_name: nav_msgs/msg/Odometry\n  gz_type_name: gz.msgs.Odometry\n  direction: GZ_TO_ROS\n'
                % (i, i, i, i))
    open(os.path.join(CFG, 'bridge_mcity_vla.yaml'), 'w').write(out)


def street(sc_id, st):
    """Indian-street clutter (config/street_indian.yaml) for the mcity_vla worlds."""
    skip, out = set(st.get('exclude', {}).get(sc_id, [])), []
    if 'pedestrians' not in skip:
        out += [_walker(p, st['walk_dae']) + _ped_body(p) for p in st.get('pedestrians', [])]
    if 'cows' not in skip:
        out += [_include(c['id'], st['cow_uri'], *c['pose'], z=0.0) for c in st.get('cows', [])]
    if 'dogs' not in skip:
        out += [_include(d['id'], 'model://street_dog', *d['pose'], speed=d.get('speed')) for d in st.get('dogs', [])]

    def kinds(v):
        return 'model://auto_rickshaw' if v['kind'] == 'auto' else 'model://traffic_car_' + v.get('color', 'grey')
    if 'parked' not in skip:
        out += [_include(v['id'], kinds(v), *v['pose']) for v in st.get('parked', [])]
    if 'traffic' not in skip:
        out += [_include(v['id'], kinds(v), *v['pose'], speed=v['speed']) for v in st.get('traffic', [])]
    return ''.join(out)


def mcity_include(mc):
    """Place the city_vla mesh (files unchanged) so mesh point junction_mesh lands on junction x1 = (105, 0)."""
    sc, (jx, jy), yaw = mc['scale'], mc['junction_mesh'], math.radians(mc['yaw_deg'])
    c, s = math.cos(yaw), math.sin(yaw)
    tx = 105.0 - (c * sc * jx - s * sc * jy)
    ty = 0.0 - (s * sc * jx + c * sc * jy)
    return ('<model name="city_vla"><static>true</static><pose>%.4f %.4f 0 0 0 %.6f</pose><link name="link">'
            '<visual name="visual"><geometry><mesh><uri>model://city_vla/meshes/mcity.obj</uri>'
            '<scale>%g %g %g</scale></mesh></geometry></visual></link></model>\n' % (tx, ty, yaw, sc, sc, sc))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mirror-signs', action='store_true')
    ap.add_argument('--offline', action='store_true')
    a = ap.parse_args()
    town = yaml.safe_load(open(os.path.join(CFG, 'town.yaml')))
    os.makedirs(WORLDS, exist_ok=True)
    mc = yaml.safe_load(open(os.path.join(CFG, 'mcity_vla.yaml')))
    st = yaml.safe_load(open(os.path.join(CFG, 'street_indian.yaml')))
    rd, mrd = roads(town), mcity_include(mc) + buildings(mc)
    bridge_mcity(st)
    for f in sorted(glob.glob(os.path.join(CFG, 'scenarios', '*.yaml'))):
        sc = yaml.safe_load(open(f))
        for s in sc.get('signs', []):
            sign_model('%s_%s' % (sc['id'], s['id']), s['text'], a.mirror_signs)
        x, y, yaw = sc['spawn']
        for suffix, r, m, gz, wn in (('', rd, None, 0.0, 'cogndn_town'),
                                     ('_mcity_vla', mrd + street(sc['id'], st), mc, -0.02, 'mcity_vla_world')):
            open(os.path.join(WORLDS, sc['id'] + suffix + '.sdf'), 'w').write(
                WORLD.format(sid=sc['id'], roads=r, objects=objects(sc, a.offline, m), x=x, y=y, yaw=yaw, gz=gz,
                             wname=wn))
            print('wrote', sc['id'] + suffix + '.sdf')


if __name__ == '__main__':
    main()
