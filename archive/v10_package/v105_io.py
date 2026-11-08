"""Read-only V10.5 telemetry boundary.

World body state is diagnostic/safety data, not wheel odometry, SLAM, or a
steering observation. All tensor reads reuse views attached by the application.
"""
import math
import numpy as np


def _host(value):
    for method in ('detach', 'cpu', 'numpy'):
        function = getattr(value, method, None)
        if callable(function):
            value = function()
    return np.asarray(value)


def _plain(value):
    """JSON-compatible values; unavailable/nonfinite values remain explicit."""
    if isinstance(value, np.bool_):
        return bool(value)
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [_plain(v) for v in value]
    try:
        return [_plain(v) for v in value]
    except TypeError:
        return str(value)


def _graph_get(app, suffix):
    path = app.paths['controller'] + '.' + suffix
    attribute = app.og.Controller.attribute(path)
    if not attribute.is_valid():
        raise RuntimeError('Graph attribute unavailable: ' + path)
    return _plain(app.og.Controller.get(attribute))


class WheelContactRecorder:
    """Read contact tensors from the already attached simulation view.

    No applied force/target/pose writes. Sample on each completed physics step;
    impulse-to-force conversion uses physics dt, never the telemetry interval.
    """
    WHEELS = {'left_drive_wheel_link','right_drive_wheel_link',
              'front_left_caster_wheel_link','front_right_caster_wheel_link',
              'rear_left_caster_wheel_link','rear_right_caster_wheel_link'}

    def __init__(self, simulation_view, root, floor_path, physics_hz):
        self.dt = 1./float(physics_hz)
        if not math.isfinite(self.dt) or self.dt <= 0:
            raise ValueError('Invalid contact physics dt')
        self.view = simulation_view.create_rigid_contact_view(root+'/*wheel_link', [floor_path])
        self.paths = [str(x) for x in self.view.sensor_paths]
        if self.view.sensor_count != 6 or len(self.paths) != 6 or {x.rsplit('/',1)[-1] for x in self.paths} != self.WHEELS:
            raise RuntimeError('Contact sensors did not resolve exactly the six AGV wheel bodies: '+str(self.paths))
        # Some backends expand the single floor filter to one nested row per
        # wheel. The outer row count is NOT the number of filter columns.
        # Preserve that mapping and verify every row; do not stringify lists.
        self.raw_filter_paths = _plain(self.view.filter_paths)
        rows = self.raw_filter_paths
        if not isinstance(rows, list) or not rows or self.view.filter_count != 1:
            raise RuntimeError('Invalid floor contact filter columns: '+str(rows))
        if all(isinstance(row, str) for row in rows):
            rows = [[row] for row in rows]
        if (len(rows) not in (1, len(self.paths)) or
                any(not isinstance(row, list) or len(row) != 1 or row[0] != str(floor_path)
                    for row in rows)):
            raise RuntimeError('Each wheel must resolve the configured floor filter: '+str(self.raw_filter_paths))
        self.filter_paths_by_sensor = rows * len(self.paths) if len(rows) == 1 else rows
        self.filters = [str(floor_path)]
        self.samples = []
        self.error_count = 0
        self.first_error = None
        self.ever_positive = False

    def sample(self, now, command):
        try:
            net = _host(self.view.get_net_contact_forces(self.dt)).astype(float).copy()
            floor = _host(self.view.get_contact_force_matrix(self.dt)).astype(float).copy()
            if net.shape != (6,3) or floor.shape != (6,1,3) or not np.isfinite(np.r_[net.ravel(),floor.ravel()]).all():
                raise RuntimeError('Invalid wheel contact tensor shape or values')
            self.ever_positive |= bool(np.max(np.linalg.norm(floor[:,0,:],axis=1)) > 1.)
            # Keep native reported signs and axis order; do not turn missing
            # measurements into zero normal loads or feed them to controls.
            self.samples.append((float(now),net,floor[:,0,:],tuple(command)))
        except Exception as exc:
            self.error_count += 1
            if self.first_error is None:
                self.first_error = type(exc).__name__+': '+str(exc)

    def metadata(self):
        return dict(recorder_revision='v10.5.5.1-per-sensor-floor-filter',
                    sensor_paths=self.paths,filter_paths=self.filters,
                    raw_filter_paths=self.raw_filter_paths,filter_paths_by_sensor=self.filter_paths_by_sensor,
                    physics_dt_s=self.dt,
                    valid_samples=len(self.samples),error_count=self.error_count,
                    first_error=self.first_error,ever_nonzero_floor_contact=self.ever_positive,
                    coordinates='world XYZ; keep backend signs; SI for this metre/kg stage',
                    caveat='A zero is not proof of loss of support unless this view has demonstrated valid positive contacts. Floor force is aggregate reported contact force, not isolated motor torque.')


def sample_diagnostics(app, now):
    """Read a physics-step snapshot without writing any wheel or stage state.

    Output wheel indices/signs come from EncoderReader, not guessed names.
    DifferentialController output is preserved in its declared controller order;
    it is deliberately not mislabeled as logical-front wheel targets.
    """
    record = dict(physics_s=float(now), source='live_attached_PhysX_views',
                  world_pose_policy='DIAGNOSTIC_ONLY; never an odom/SLAM/steering input',
                  graph_output_order='DifferentialController declared left,right; joint binding is separately recorded',
                  graph_output_timing='Observed at sample phase; not proof that this target was applied in the just-completed physics step',
                  actual_logical_wheel_positions_rad=None,
                  actual_logical_wheel_velocities_rad_s=None,
                  graph_wheel_targets_rad_s=None, body=None, errors={})
    encoders = getattr(app, 'encoders', None)
    if encoders is not None:
        try:
            indices = [int(encoders.left), int(encoders.right)]
            signs = np.asarray([encoders.lsign, encoders.rsign], dtype=float)
            names = list(encoders.view.shared_metatype.dof_names)
            record['logical_wheel_joint_names'] = [names[i] for i in indices]
            record['logical_encoder_signs'] = signs.tolist()
            record['joint_force_observations'] = {}
            for method in ('get_dof_projected_joint_forces', 'get_dof_actuation_forces'):
                try:
                    values = _host(getattr(encoders.view, method)())
                    if values.shape != (1,len(names)) or not np.isfinite(values).all():
                        raise RuntimeError('Invalid all-DOF force observation')
                    record['joint_force_observations'][method] = values[0].astype(float).tolist()
                except Exception as exc:
                    record['errors']['joint_force_observations.'+method] = type(exc).__name__+': '+str(exc)
            record['joint_force_observation_policy'] = 'Projected joint force is generalized incoming force, not isolated motor-drive torque; actuation forces are commanded effort, not proof of actual drive torque.'
            # Include all DOFs to identify accidental caster commands and the
            # passive caster angles without introducing another command writer.
            record['all_joint_names'] = names
            record['all_joint_state_raw'] = {}
            for method in ('get_dof_positions', 'get_dof_velocities', 'get_dof_velocity_targets'):
                try:
                    values = _host(getattr(encoders.view, method)())
                    if values.shape != (1, len(names)) or not np.isfinite(values).all():
                        raise RuntimeError('Expected finite one-articulation all-DOF state')
                    record['all_joint_state_raw'][method] = values[0].astype(float).tolist()
                except Exception as exc:
                    record['errors']['all_joint_state_raw.'+method] = type(exc).__name__+': '+str(exc)
            for method, key in [('get_dof_positions', 'actual_logical_wheel_positions_rad'),
                                ('get_dof_velocities', 'actual_logical_wheel_velocities_rad_s')]:
                try:
                    values = _host(getattr(encoders.view, method)())
                    if values.ndim != 2 or values.shape[0] != 1:
                        raise RuntimeError('Expected one articulation tensor row')
                    logical = values[0, indices].astype(float) * signs
                    if not np.isfinite(logical).all():
                        raise RuntimeError('Nonfinite live joint sample')
                    record[key] = logical.tolist()
                except Exception as exc:
                    record['errors'][key] = type(exc).__name__ + ': ' + str(exc)
        except Exception as exc:
            record['errors']['encoder_binding'] = type(exc).__name__ + ': ' + str(exc)
    else:
        record['errors']['encoder_binding'] = 'EncoderReader is not attached'
    try:
        targets = _graph_get(app, 'outputs:velocityCommand')
        if not isinstance(targets, list) or len(targets) != 2 or any(v is None for v in targets):
            raise RuntimeError('Expected two finite DifferentialController wheel targets')
        record['graph_wheel_targets_rad_s'] = targets
        record['graph_sample_physics_s'] = float(now)
    except Exception as exc:
        record['errors']['graph_output'] = type(exc).__name__ + ': ' + str(exc)
    scanner = getattr(app, 'scanner', None)
    view = getattr(scanner, 'body_view', None)
    if view is not None:
        try:
            pose = _host(view.get_transforms())[0].astype(float)
            velocity = _host(view.get_velocities())[0].astype(float)
            if pose.shape != (7,) or velocity.shape != (6,) or not np.isfinite(np.r_[pose, velocity]).all():
                raise RuntimeError('Invalid live base-body pose/velocity')
            norm = float(np.linalg.norm(pose[3:7]))
            if norm < 1e-8:
                raise RuntimeError('Zero quaternion in body diagnostic')
            rotation = scanner.rotation_row(pose[3:7] / norm)
            front = np.eye(3)
            if app.cfg['front_axis_sign'] < 0:
                front[0, 0] = front[1, 1] = -1.
            front_rotation = front @ rotation
            units = float(scanner.units)
            record['body'] = dict(physics_s=float(now),
                world_position_m=(pose[:3]*units).tolist(),
                world_quaternion_xyzw=(pose[3:7]/norm).tolist(),
                physical_front_world_yaw_rad=float(math.atan2(front_rotation[0, 1], front_rotation[0, 0])),
                world_linear_velocity_m_s=(velocity[:3]*units).tolist(),
                world_angular_velocity_rad_s=velocity[3:].tolist(),
                linear_front_m_s=(velocity[:3] @ front_rotation.T*units).tolist(),
                angular_front_rad_s=(velocity[3:] @ front_rotation.T).tolist(),
                upright_cosine=float(front_rotation[2, 2]),
                tilt_deg=float(np.rad2deg(np.arccos(np.clip(front_rotation[2, 2], -1., 1.)))),
                usage='DIAGNOSTIC/SAFETY ONLY; no world pose feedback to navigation')
        except Exception as exc:
            record['errors']['body'] = type(exc).__name__ + ': ' + str(exc)
    else:
        record['errors']['body'] = 'Sensor base-body view not attached; no stale sample substituted'
    return _plain(record)


def snapshot_effective_physics(app):
    """One-time effective USD/graph readings. Does not apply APIs or edit layers."""
    from pxr import Usd, UsdPhysics, UsdShade, UsdGeom
    stage = app.stage
    base = stage.GetPrimAtPath(app.paths['base'])
    root = base.GetParent()
    result = dict(revision='v10.5-readonly-effective-settings',
        physics_s=float(app.simulation_time()),
        meters_per_unit=float(UsdGeom.GetStageMetersPerUnit(stage)),
        stage_identifier=stage.GetRootLayer().identifier,
        session_sublayers=list(stage.GetSessionLayer().subLayerPaths),
        base=app.paths['base'], controller=app.paths['controller'],
        software_body_config=dict(app.cfg.get('body', {})),
        graph={}, joints=[], bodies=[], colliders=[], physics_scenes=[], errors={},
        policy='Read-only resolved values; missing schema attributes are not hardware calibration')
    for name in ['wheelRadius', 'wheelDistance', 'maxWheelSpeed', 'maxLinearSpeed',
                 'maxAngularSpeed', 'maxAcceleration', 'maxDeceleration',
                 'maxAngularAcceleration', 'dt']:
        try:
            result['graph'][name] = _graph_get(app, 'inputs:'+name)
        except Exception as exc:
            result['errors']['graph.'+name] = str(exc)
    # Capture downstream joint-name/target bindings without inventing array order.
    graph_root = stage.GetPrimAtPath(str(stage.GetPrimAtPath(app.paths['controller']).GetParent().GetPath()))
    bindings = []
    if graph_root.IsValid():
        for node in Usd.PrimRange(graph_root):
            values = {}
            for attr in node.GetAttributes():
                if attr.GetName() in ('inputs:jointNames', 'inputs:jointIndices', 'inputs:robotPath',
                                      'inputs:velocityCommand', 'inputs:positionCommand', 'inputs:effortCommand'):
                    values[attr.GetName()] = _plain(attr.Get())
                    values[attr.GetName()+'_connections'] = [str(x) for x in attr.GetConnections()]
                    try:
                        values[attr.GetName()+'_evaluated'] = _plain(app.og.Controller.get(app.og.Controller.attribute(str(attr.GetPath()))))
                    except Exception as exc:
                        values[attr.GetName()+'_evaluation_error'] = str(exc)
            for rel in node.GetRelationships():
                if rel.GetName() in ('inputs:targetPrim', 'inputs:robotPrim'):
                    values[rel.GetName()] = [str(x) for x in rel.GetTargets()]
            if values:
                bindings.append(dict(path=str(node.GetPath()), **values))
    result['articulation_command_bindings'] = bindings
    for prim in Usd.PrimRange(root):
        path = str(prim.GetPath())
        if prim.IsA(UsdPhysics.Joint):
            attrs = {}
            for attr in prim.GetAttributes():
                name = attr.GetName()
                if name.startswith(('physics:', 'physxJoint:', 'drive:')):
                    attrs[name] = _plain(attr.Get())
            rels = {rel.GetName(): [str(x) for x in rel.GetTargets()]
                    for rel in prim.GetRelationships() if rel.GetName() in ('physics:body0', 'physics:body1')}
            result['joints'].append(dict(path=path, attributes=attrs, relationships=rels))
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            attrs = {attr.GetName(): _plain(attr.Get()) for attr in prim.GetAttributes()
                     if attr.GetName().startswith(('physics:', 'physxRigidBody:', 'physxArticulation:'))}
            result['bodies'].append(dict(path=path, attributes=attrs))
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            attrs = {attr.GetName(): _plain(attr.Get()) for attr in prim.GetAttributes()
                     if attr.GetName().startswith(('physics:', 'physxCollision:'))}
            item = dict(path=path, type_name=str(prim.GetTypeName()), attributes=attrs)
            try:
                material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
                if material:
                    item['material_path'] = str(material.GetPath())
                    mat = material.GetPrim()
                    item['material_attributes'] = {a.GetName(): _plain(a.Get()) for a in mat.GetAttributes()
                        if a.GetName().startswith(('physics:', 'physxMaterial:'))}
                else:
                    item['material_path'] = None
            except Exception as exc:
                item['material_read_error'] = str(exc)
            result['colliders'].append(item)
            try:
                # Static shape geometry is measured relative to its owning
                # rigid body; world position is diagnostic only.
                cache = UsdGeom.XformCache(Usd.TimeCode.Default())
                bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default','render','proxy']).ComputeWorldBound(prim).ComputeAlignedRange()
                units = result['meters_per_unit']
                item['world_aabb_m'] = [np.asarray(bounds.GetMin())*units, np.asarray(bounds.GetMax())*units]
                owner = prim
                while owner.IsValid() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
                    owner = owner.GetParent()
                item['owning_rigid_body'] = str(owner.GetPath()) if owner.IsValid() else None
                if owner.IsValid() and 'wheel' in owner.GetName().lower() and prim.IsA(UsdGeom.Cylinder):
                    cylinder = UsdGeom.Cylinder(prim)
                    matrix = (np.array(cache.GetLocalToWorldTransform(prim)) @
                              np.linalg.inv(np.array(cache.GetLocalToWorldTransform(owner))))
                    item['primitive_cylinder'] = dict(
                        radius_m=float(cylinder.GetRadiusAttr().Get())*units,
                        width_m=float(cylinder.GetHeightAttr().Get())*units,
                        axis=str(cylinder.GetAxisAttr().Get()), local_to_owner=matrix.tolist())
                    try:
                        import carb
                        from omni.physx.bindings._physx import SETTING_COLLISION_APPROXIMATE_CYLINDERS
                        item['cylinder_convex_approximation_setting'] = carb.settings.get_settings().get(SETTING_COLLISION_APPROXIMATE_CYLINDERS)
                    except Exception as exc:
                        item['cylinder_setting_read_error'] = type(exc).__name__+': '+str(exc)
                if owner.IsValid() and 'wheel' in owner.GetName().lower() and prim.IsA(UsdGeom.Mesh):
                    points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
                    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
                        raise RuntimeError('No wheel collider vertices to measure')
                    matrix = (np.array(cache.GetLocalToWorldTransform(prim)) @
                              np.linalg.inv(np.array(cache.GetLocalToWorldTransform(owner))))
                    local = (points @ matrix[:3,:3]+matrix[3,:3])*units
                    item['wheel_vertices_owner_local_m'] = local
                    item['wheel_local_aabb_m'] = [local.min(0), local.max(0)]
                    radial = np.linalg.norm(local[:,[0,2]], axis=1)
                    item['radial_extent_about_owner_Y_m'] = [float(radial.min()), float(radial.max())]
            except Exception as exc:
                item['geometry_read_error'] = type(exc).__name__+': '+str(exc)
    # These are resolved runtime tensors, unlike USD's automatic-inertia sentinel.
    result['resolved_link_properties'] = {}
    encoders = getattr(app, 'encoders', None)
    if encoders is not None:
        for name in ('get_masses', 'get_inertias', 'get_coms'):
            try:
                result['resolved_link_properties'][name] = _plain(_host(getattr(encoders.view, name)()))
            except Exception as exc:
                result['errors']['resolved.'+name] = type(exc).__name__+': '+str(exc)
        try:
            result['resolved_link_properties']['link_names'] = list(encoders.view.shared_metatype.link_names)
        except Exception as exc:
            result['errors']['resolved.link_names'] = str(exc)
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            result['physics_scenes'].append(dict(path=str(prim.GetPath()), attributes={
                a.GetName(): _plain(a.Get()) for a in prim.GetAttributes()
                if a.GetName().startswith(('physics:', 'physxScene:'))}))
    return _plain(result)

