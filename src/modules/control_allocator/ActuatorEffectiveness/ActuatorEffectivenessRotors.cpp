/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ActuatorEffectivenessRotors.hpp
 *
 * Actuator effectiveness computed from rotors position and orientation
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessRotors.hpp"

#include "ActuatorEffectivenessTilts.hpp"

using namespace matrix;

ActuatorEffectivenessRotors::ActuatorEffectivenessRotors(ModuleParams *parent, AxisConfiguration axis_config,
		bool tilt_support)
	: ModuleParams(parent), _axis_config(axis_config), _tilt_support(tilt_support)
{
	for (int i = 0; i < NUM_ROTORS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PX", i);		// 提取电机位置参数  存储于 CA_ROTORx_PX, CA_ROTORx_PY, ...
		_param_handles[i].position_x = param_find(buffer);			// _param_handles 相当于管理电机参数句柄的接口
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PY", i);		// param_find 函数得到的是参数的句柄（ID），而不是参数的值
		_param_handles[i].position_y = param_find(buffer);			// 得到参数句柄后，可以使用 param_get 函数来获取参数的值
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PZ", i);
		_param_handles[i].position_z = param_find(buffer);

		if (_axis_config == AxisConfiguration::Configurable) {		// 提取电机轴向参数  存储于 CA_ROTORx_AX, CA_ROTORx_AY, CA_ROTORx_AZ
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AX", i);	// 电机轴向参数是表明轴向方向的单位向量
			_param_handles[i].axis_x = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AY", i);
			_param_handles[i].axis_y = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AZ", i);
			_param_handles[i].axis_z = param_find(buffer);
		}

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_CT", i);		// 设定电机推理系数
		_param_handles[i].thrust_coef = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_KM", i);		// 设定电机反扭矩系数
		_param_handles[i].moment_ratio = param_find(buffer);

		// 是否支持倾斜控制
		if (_tilt_support) {
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_TILT", i);
			_param_handles[i].tilt_index = param_find(buffer);
		}
	}

	_count_handle = param_find("CA_ROTOR_COUNT");	// 获取表征电机数量的句柄
	// _ca_aircraft_mode = param_find("CA_AIRCRAFT_MD");

	updateParams();
}

void ActuatorEffectivenessRotors::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_count_handle, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_geometry.num_rotors = math::min(NUM_ROTORS_MAX, (int)count);		// 更新无人机机型的电机数量

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		Vector3f &position = _geometry.rotors[i].position;
		param_get(_param_handles[i].position_x, &position(0));			// 更新电机位置参数 存储在 position 向量中
		param_get(_param_handles[i].position_y, &position(1));
		param_get(_param_handles[i].position_z, &position(2));

		Vector3f &axis = _geometry.rotors[i].axis;

		switch (_axis_config) {									// 根据轴向配置的类型 更新轴向信息
		case AxisConfiguration::Configurable:
			param_get(_param_handles[i].axis_x, &axis(0));		// 若轴向可配置，则获取相应的轴向参数
			param_get(_param_handles[i].axis_y, &axis(1));
			param_get(_param_handles[i].axis_z, &axis(2));
			break;

		case AxisConfiguration::FixedForward:
			axis = Vector3f(1.f, 0.f, 0.f);
			break;

		case AxisConfiguration::FixedUpwards:
			axis = Vector3f(0.f, 0.f, -1.f);		// 电机轴固定向上  z 轴负方向对应向上
			break;
		}

		param_get(_param_handles[i].thrust_coef, &_geometry.rotors[i].thrust_coef);			// 获取电机推力系数
		param_get(_param_handles[i].moment_ratio, &_geometry.rotors[i].moment_ratio);		// 获取电机反扭矩系数

		if (_tilt_support) {
			int32_t tilt_param{0};
			param_get(_param_handles[i].tilt_index, &tilt_param);
			_geometry.rotors[i].tilt_index = tilt_param - 1;			// 更新电机倾斜参数

		} else {
			_geometry.rotors[i].tilt_index = -1;
		}
	}

	// int32_t _aircraft_mode = 0;
	// if (param_get(_ca_aircraft_mode, &_aircraft_mode) != 0)
	PX4_INFO("Get aircraft mode: %d", (int)_ca_aircraft_md.get());
	// else
		// PX4_INFO("Get aircraft mode failed");

}

// 根据 configuration 配置相应的效率矩阵 （添加一组执行机构）
bool
ActuatorEffectivenessRotors::addActuators(Configuration &configuration)
{
	if (configuration.num_actuators[(int)ActuatorType::SERVOS] > 0) {		// 配置电机过程中，不应当出现舵机
		PX4_ERR("Wrong actuator ordering: servos need to be after motors");
		return false;
	}

	int num_actuators = computeEffectivenessMatrix(_geometry,
			    configuration.effectiveness_matrices[configuration.selected_matrix],	// 当前选中的效率矩阵（可以认为是选中的执行机构组）
			    configuration.num_actuators_matrix[configuration.selected_matrix]);		// 当前选中的效率矩阵的执行机构数量
	configuration.actuatorsAdded(ActuatorType::MOTORS, num_actuators);					// 表明已向系统中添加 num_actuators 个电机 （更新数量）
	return true;
}

// 更新 configuration 中的 effectiveness 矩阵
// 添加所有 geometry 中的执行机构
// actuator_start_index 表示新增的执行机构在所有执行机构中起始的序号
int ActuatorEffectivenessRotors::computeEffectivenessMatrix(const Geometry &geometry,
		EffectivenessMatrix &effectiveness, int actuator_start_index)
{
	int num_actuators = 0;

	for (int i = 0; i < geometry.num_rotors; i++) {

		if (i + actuator_start_index >= NUM_ACTUATORS) {
			break;
		}

		++num_actuators;

		// Get rotor axis
		Vector3f axis = geometry.rotors[i].axis;

		// Normalize axis
		float axis_norm = axis.norm();

		if (axis_norm > FLT_EPSILON) {
			axis /= axis_norm;

		} else {
			// Bad axis definition, ignore this rotor
			continue;
		}

		// Get rotor position
		const Vector3f &position = geometry.rotors[i].position;

		// Get coefficients
		float ct = geometry.rotors[i].thrust_coef;
		float km = geometry.rotors[i].moment_ratio;

		if (geometry.propeller_torque_disabled) {
			km = 0.f;
		}

		if (geometry.propeller_torque_disabled_non_upwards) {
			bool upwards = fabsf(axis(0)) < 0.1f && fabsf(axis(1)) < 0.1f && axis(2) < -0.5f;

			if (!upwards) {
				km = 0.f;
			}
		}

		if (fabsf(ct) < FLT_EPSILON) {
			continue;
		}

		// Compute thrust generated by this rotor
		matrix::Vector3f thrust = ct * axis;			// axis 三维中的轴向量 单位向量

		// Compute moment generated by this rotor
		matrix::Vector3f moment = ct * position.cross(axis) - ct * km * axis;		// 力产生的力矩 + 电机反扭距

		// Fill corresponding items in effectiveness matrix
		for (size_t j = 0; j < 3; j++) {		// 三个轴向
			effectiveness(j, i + actuator_start_index) = moment(j);			// 效率矩阵  控制分配矩阵是他的逆（伪逆）
			effectiveness(j + 3, i + actuator_start_index) = thrust(j);
		}

		// 判断偏航是否由其他方法/系统控制
		if (geometry.yaw_by_differential_thrust_disabled) {
			// set yaw effectiveness to 0 if yaw is controlled by other means (e.g. tilts)
			effectiveness(2, i + actuator_start_index) = 0.f;
		}

		// 判断是否为三维推力（对于四旋翼仅为 z 轴推力 对于固定翼为 x 轴推力）
		if (geometry.three_dimensional_thrust_disabled) {
			// Special case tiltrotor: instead of passing a 3D thrust vector (that would mostly have a x-component in FW, and z in MC),
			// pass the vector magnitude as z-component, plus the collective tilt. Passing 3D thrust plus tilt is not feasible as they
			// can't be allocated independently, and with the current controller it's not possible to have collective tilt calculated
			// by the allocator directly.

			effectiveness(0 + 3, i + actuator_start_index) = 0.f;
			effectiveness(1 + 3, i + actuator_start_index) = 0.f;
			effectiveness(2 + 3, i + actuator_start_index) = -ct;
		}

		// 判断是否为水平驱动电机
		if (i >= 4) {			// 假定4个之后为水平驱动电机
			unsigned int index = i;
			for (size_t j = 0; j < 3; j++) {
				effectiveness(j, i + actuator_start_index) = 0;					// 水平电机不产生纵向推力与转矩
				effectiveness(j + 3, i + actuator_start_index) = 0;

				if (j == 0) {
					effectiveness(j + 3, i + actuator_start_index) = ct;		// 推力
				} else if (j == 1) {
					effectiveness(j + 3, i + actuator_start_index) = - ct * position(1);		// 转矩
				}

			}
			PX4_INFO("Set the aux rotor %d, with the effectiveness row:", index);
			PX4_INFO("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", (double)effectiveness(0, i + actuator_start_index),
														   (double)effectiveness(1, i + actuator_start_index),
														   (double)effectiveness(2, i + actuator_start_index),
														   (double)effectiveness(0 + 3, i + actuator_start_index),
														   (double)effectiveness(1 + 3, i + actuator_start_index),
														   (double)effectiveness(2 + 3, i + actuator_start_index));
		}
	}

	// 调试输出
	PX4_INFO("The total Effectiveness Matrix:");
	for (int j = 0; j < 6; j++) {
		PX4_INFO("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f", (double)effectiveness(j, 0 + actuator_start_index),
														   (double)effectiveness(j, 1 + actuator_start_index),
														   (double)effectiveness(j, 2 + actuator_start_index),
														   (double)effectiveness(j, 3 + actuator_start_index),
														   (double)effectiveness(j, 4 + actuator_start_index),
														   (double)effectiveness(j, 5 + actuator_start_index));
	}

	return num_actuators;		// 理论上应为 geometry.num_rotors
}

uint32_t ActuatorEffectivenessRotors::updateAxisFromTilts(const ActuatorEffectivenessTilts &tilts,
		float collective_tilt_control)
{
	if (!PX4_ISFINITE(collective_tilt_control)) {
		collective_tilt_control = -1.f;
	}

	uint32_t nontilted_motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		int tilt_index = _geometry.rotors[i].tilt_index;

		if (tilt_index == -1 || tilt_index >= tilts.count()) {
			nontilted_motors |= 1u << i;
			continue;
		}

		const ActuatorEffectivenessTilts::Params &tilt = tilts.config(tilt_index);
		const float tilt_angle = math::lerp(tilt.min_angle, tilt.max_angle, (collective_tilt_control + 1.f) / 2.f);
		const float tilt_direction = math::radians((float)tilt.tilt_direction);
		_geometry.rotors[i].axis = tiltedAxis(tilt_angle, tilt_direction);
	}

	return nontilted_motors;
}

Vector3f ActuatorEffectivenessRotors::tiltedAxis(float tilt_angle, float tilt_direction)
{
	Vector3f axis{0.f, 0.f, -1.f};
	return Dcmf{Eulerf{0.f, -tilt_angle, tilt_direction}} * axis;
}

uint32_t ActuatorEffectivenessRotors::getMotors() const		// 获取所有电机掩码 对应位为 1
{
	uint32_t motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		motors |= 1u << i;
	}

	return motors;
}

uint32_t ActuatorEffectivenessRotors::getUpwardsMotors() const		// 获取所有方向向上的电机掩码
{
	uint32_t upwards_motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		const Vector3f &axis = _geometry.rotors[i].axis;

		if (fabsf(axis(0)) < 0.1f && fabsf(axis(1)) < 0.1f && axis(2) < -0.5f) {
			upwards_motors |= 1u << i;
		}
	}

	return upwards_motors;
}

uint32_t ActuatorEffectivenessRotors::getForwardsMotors() const
{
	uint32_t forward_motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		const Vector3f &axis = _geometry.rotors[i].axis;

		if (axis(0) > 0.5f && fabsf(axis(1)) < 0.1f && fabsf(axis(2)) < 0.1f) {
			forward_motors |= 1u << i;
		}
	}

	return forward_motors;
}

bool
ActuatorEffectivenessRotors::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}
	PX4_INFO("[Rotors] Configuration after adding actuators: Selected Matrix %d", configuration.selected_matrix);
	return addActuators(configuration);
}
