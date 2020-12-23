#! /usr/bin/env python

import rospy

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

from pkg_vb_sim.srv import ConveyorBeltControl
from pkg_vb_sim.srv import ConveyorBeltControlRequest
from pkg_vb_sim.srv import ConveyorBeltControlResponse

from pkg_vb_sim.msg import LogicalCameraImage

from gazebo_msgs.srv import ApplyBodyWrench, GetModelProperties, GetWorldProperties, SetModelState
from geometry_msgs.msg import Wrench

'''

	def on_apply_wrench_clicked_(self):
		success = True
		apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
		body_name = self._widget.comboBoxObjectList.currentText()
		wrench = Wrench()
		wrench.force.x = float(str(self._widget.lineEdit.text()))
		wrench.force.y = float(str(self._widget.lineEdit_2.text()))
		wrench.force.z = float(str(self._widget.lineEdit_3.text()))

		try:
			resp1 = apply_body_wrench(body_name, "", None, wrench, rospy.Time.from_sec(0), rospy.Duration.from_sec(1.0))
		except rospy.ServiceException:
			success = False
		if success:
			if not resp1.success:
				success = False

		if not success:
			QMessageBox.warning(self._widget, "Warning", "Could not apply wrench to selected object.")

'''


class ConveyorBelt():

	# Constructor
	def __init__(self):
		rospy.loginfo(
			'\033[94m' + " >>> Conveyor Belt Power init done." + '\033[0m')

	def apply_force(self, arg_body_name, arg_x, arg_y, arg_z):
		success = True
		apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
		body_name = arg_body_name
		wrench = Wrench()
		wrench.force.x = arg_x
		wrench.force.y = arg_y
		wrench.force.z = arg_z

		try:
			resp1 = apply_body_wrench(body_name, "", None, wrench, rospy.Time.from_sec(0), rospy.Duration.from_sec(1.0))
		except rospy.ServiceException:
			success = False
		if success:
			if not resp1.success:
				success = False


	def callback_service_on_request(self, req):
		rospy.loginfo(
			'\033[94m' + " >>> Request Rx - Conveyor Belt Power: {}".format(req.power) + '\033[0m')

		set_power = req.power
		# OldMax = 100
		# OldMin = 0
		# NewMax = 100
		# NewMin = 60
		# OldValue = req.power
		# if(req.power == 0):
		# 	set_power = 0
		# else:
		# 	OldRange = (OldMax - OldMin)  
		# 	NewRange = (NewMax - NewMin)  
		# 	NewValue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin
		# 	set_power = NewValue
		
		print(">>>> {} <<<<".format(set_power))

		gazebo_conveyor_belt_service_proxy = rospy.ServiceProxy(
			'/gazebo_sim/conveyor/control', ConveyorBeltControl)
		service_request = ConveyorBeltControlRequest(set_power)
		service_response = gazebo_conveyor_belt_service_proxy(service_request)
		
		self.apply_force("conveyor_belt::conveyor_belt_moving::belt", 0, 0, 9.8)

		self.apply_force("packagen1::link", 0, 0, 0.000000001)
		self.apply_force("packagen2::link", 0, 0, 0.000000001)
		self.apply_force("packagen3::link", 0, 0, 0.000000001)
		

		return conveyorBeltPowerMsgResponse(service_response.success)

	# Destructor
	def __del__(self):
		rospy.loginfo(
			'\033[94m' + " >>> Conveyor Belt Power delete." + '\033[0m')


def main():
	rospy.init_node('node_service_server_conveyor_belt')

	warehouse_conveyor = ConveyorBelt()

	rospy.wait_for_service('/gazebo_sim/conveyor/control')

	s = rospy.Service('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg,
					  warehouse_conveyor.callback_service_on_request)

	rospy.loginfo(
		'\033[94m' + " >>> Conveyor Belt Power Service Ready." + '\033[0m')

	rospy.spin()


if __name__ == "__main__":
	main()
