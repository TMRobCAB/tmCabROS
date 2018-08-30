/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rqt_tm_operation/tm_operation.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>

using namespace std;

namespace rqt_tm_operation {

TmOperation::TmOperation() :
		jntCtlSelected_(true), sysRunning_(false), jointIdxRunning_(0), rqt_gui_cpp::Plugin(), widget_(
				0) {
	setObjectName("TmOperation");
}

void TmOperation::initPlugin(qt_gui_cpp::PluginContext& context) {
	widget_ = new QWidget();
	ui_.setupUi(widget_);

	if (context.serialNumber() > 1) {
		widget_->setWindowTitle(
				widget_->windowTitle() + " ("
						+ QString::number(context.serialNumber()) + ")");
	}
	context.addWidget(widget_);

	qRegisterMetaType<beginner_tutorials::SYS_STATUS>(
			"beginner_tutorials::SYS_STATUS");

	ros::NodeHandle n = getNodeHandle();

	sysStatS_ = n.subscribe("/sys_status", 1, &TmOperation::callbackSysStatus,
			this);

	connect(this,
			SIGNAL(processStatusUpdate(const beginner_tutorials::SYS_STATUS&)),
			this,
			SLOT(sysStatusUpdated(const beginner_tutorials::SYS_STATUS&)));

	// TM Parameters Tab Initialization
	//////////////////////////////////////////////

	ui_.j1_stat_textBrowser->setText("Waiting for node");
	ui_.j2_stat_textBrowser->setText("Waiting for node");
	ui_.j3_stat_textBrowser->setText("Waiting for node");

	ui_.j_sel_comboBox->setCurrentIndex(ui_.j_sel_comboBox->findText("All"));

	connect(ui_.j_sel_comboBox, SIGNAL(activated(int)), this,
			SLOT(EnableIdx(int)));

	ui_.start_idx_pushButton->setEnabled(false);

	connect(ui_.ack_Safe_pushButton, SIGNAL(clicked()), this,
			SLOT(ackSafety()));

	ui_.safe_pars_comboBox->setCurrentIndex(
			ui_.safe_pars_comboBox->findText("Position"));
	connect(ui_.set_safe_par_pushButton, SIGNAL(clicked()), this,
			SLOT(setSafety()));

	ui_.ctl_pars_comboBox->setCurrentIndex(
			ui_.ctl_pars_comboBox->findText("Position PID"));
	ui_.set_par_stackedWidget->setCurrentIndex(
			ui_.set_par_stackedWidget->indexOf(ui_.pid_page));
	connect(ui_.ctl_pars_comboBox, SIGNAL(activated (const QString &)), this,
			SLOT(ctlParSelected (const QString &)));
	connect(ui_.set_ctl_par_pushButton, SIGNAL(clicked()), this,
			SLOT(setCtlPars()));

	connect(ui_.start_idx_pushButton, SIGNAL(clicked()), this,
			SLOT(toggelIndex()));
	connect(ui_.p_idx_ref_pushButton, SIGNAL(pressed()), this,
			SLOT(toggelIndexPosRef()));
	connect(ui_.p_idx_ref_pushButton, SIGNAL(released()), this,
			SLOT(toggelIndexPosRef()));
	connect(ui_.n_idx_ref_pushButton, SIGNAL(pressed()), this,
			SLOT(toggelIndexNegRef()));
	connect(ui_.n_idx_ref_pushButton, SIGNAL(released()), this,
			SLOT(toggelIndexNegRef()));

	// TM Operation Tab Initialization
	//////////////////////////////////////////////

	connect(ui_.start_op_pushButton, SIGNAL(clicked()), this,
			SLOT(toggelOperation()));

	ui_.op_mode_comboBox->setCurrentIndex(
			ui_.op_mode_comboBox->findText("PTP"));
	connect(ui_.sel_op_mode_pushButton, SIGNAL(clicked()), this,
			SLOT(selOpMode()));

	ui_.op_panel_stackedWidget->setCurrentIndex(
			ui_.op_panel_stackedWidget->indexOf(ui_.ptp_page));

	connect(ui_.g_comp_checkBox, SIGNAL(stateChanged (int)), this,
			SLOT(toggleGComp (int)));

	connect(ui_.idx_pose_checkBox, SIGNAL(stateChanged (int)), this,
				SLOT(toggleIdxPose (int)));

	connect(ui_.exe_ptp_pushButton, SIGNAL(clicked()), this, SLOT(ptpExe()));

	connect(ui_.home_pushButton, SIGNAL(clicked()), this, SLOT(goHome()));

	connect(ui_.exe_jnt_traj_pushButton, SIGNAL(clicked()), this,
			SLOT(jntTrajExe()));

	connect(ui_.update_sens_k_pushButton, SIGNAL(clicked()), this,
			SLOT(updateSensK()));

	connect(ui_.update_scale_k_pushButton, SIGNAL(clicked()), this,
				SLOT(updateScaleK()));

	connect(ui_.ptp_mode_comboBox, SIGNAL(activated (const QString &)), this,
			SLOT(ptpModLabels(const QString &)));

	connect(ui_.traj_var_comboBox, SIGNAL(activated (const QString &)), this,
			SLOT(trajVarSelected(const QString &)));

	ui_.traj_steps_label->setVisible(false);
	ui_.traj_num_steps_plainTextEdit->setVisible(false);

}

void TmOperation::shutdownPlugin() {
	sysStatS_.shutdown();

	posRefPub_.shutdown();

	setGCompPub_.shutdown();
}

void TmOperation::saveSettings(qt_gui_cpp::Settings& plugin_settings,
		qt_gui_cpp::Settings& instance_settings) const {
//  QString topic = ui_.topics_combo_box->currentText();
//  //qDebug("TmOperation::saveSettings() topic '%s'", topic.toStdString().c_str());
//  instance_settings.setValue("topic", topic);
//  instance_settings.setValue("zoom1", ui_.zoom_1_push_button->isChecked());
//  instance_settings.setValue("dynamic_range", ui_.dynamic_range_check_box->isChecked());
//  instance_settings.setValue("max_range", ui_.max_range_double_spin_box->value());
}

void TmOperation::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
		const qt_gui_cpp::Settings& instance_settings) {
//  bool zoom1_checked = instance_settings.value("zoom1", false).toBool();
//  ui_.zoom_1_push_button->setChecked(zoom1_checked);
//
//  bool dynamic_range_checked = instance_settings.value("dynamic_range", false).toBool();
//  ui_.dynamic_range_check_box->setChecked(dynamic_range_checked);
//
//  double max_range = instance_settings.value("max_range", ui_.max_range_double_spin_box->value()).toDouble();
//  ui_.max_range_double_spin_box->setValue(max_range);
//
//  QString topic = instance_settings.value("topic", "").toString();
//  //qDebug("TmOperation::restoreSettings() topic '%s'", topic.toStdString().c_str());
//  selectTopic(topic);
}

void TmOperation::ackSafety() {

	ros::ServiceClient client;

	stringstream srvName;
	beginner_tutorials::SAFE_STATE srv;

	int _jointNum = ui_.j_sel_comboBox->currentIndex();

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");
		srvName << "/joint_" << i << "_set_safe";

		client = getNodeHandle().serviceClient<beginner_tutorials::SAFE_STATE>(
				srvName.str());

		if (client.call(srv)) {

			if (srv.response.ack)
				ROS_INFO("Joint %d safety acknowledge", i);
			else
				ROS_WARN("Joint %d already safe", i);

		} else {

			ROS_ERROR("Error Calling joint_%d safe acknowledge", i);

			//return -1;
		}
	}
}

void TmOperation::setSafety() {

	const string minVal(
			ui_.safe_par_min_val_plainTextEdit->toPlainText().toStdString());
	const string maxVal(
			ui_.safe_par_max_val_plainTextEdit->toPlainText().toStdString());

	if (!isFloat(minVal) || !isFloat(minVal)) {

		ROS_ERROR("Arguments must be floats");

		return;
	}

	const QString safeName(ui_.safe_pars_comboBox->currentText());
	const QString ctlName(ui_.Ctl_Sel_comboBox->currentText());

	ros::NodeHandle n = getNodeHandle();
	ros::ServiceClient client;

	stringstream srvName;
	int _jointNum = ui_.j_sel_comboBox->currentIndex();

	bool success = true;

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");

		srvName << "/joint_" << i << "_select_Ctl";

		client = n.serviceClient<beginner_tutorials::SELECT_CTL>(srvName.str());

		beginner_tutorials::SELECT_CTL selSrv;

		if (ctlName.contains("Pos"))
			selSrv.request.id =
					beginner_tutorials::SELECT_CTL::Request::POS_CTL;

		else if (ctlName.contains("Speed"))
			selSrv.request.id =
					beginner_tutorials::SELECT_CTL::Request::SPEED_CTL;

		else if (ctlName.contains("Force"))
			selSrv.request.id =
					beginner_tutorials::SELECT_CTL::Request::FORCE_CTL;

		else if (ctlName.contains("Curr")) {

			ROS_ERROR("Current Control To be Implemented...");

			return;
		} else if (ctlName.contains("Break"))
			selSrv.request.id =
					beginner_tutorials::SELECT_CTL::Request::BREAK_CTL;

		else if (ctlName.contains("Idx"))
			selSrv.request.id =
					beginner_tutorials::SELECT_CTL::Request::IDX_CTL;

		if (client.call(selSrv)) {

			if (!selSrv.response.ack) {
				ROS_ERROR("Unable to  select Joint %d %s scheme", i,
						ctlName.toStdString().data());

				continue;
			}
		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

			continue;
		}

		srvName.str("");

		if (safeName.compare("Position") == 0) {

			beginner_tutorials::SAFE_POS srv;
			srvName << "/joint_" << i << "_set_safe_Pos";

			client = n.serviceClient<beginner_tutorials::SAFE_POS>(
					srvName.str());

			srv.request.maxPos = atof(maxVal.data());
			srv.request.minPos = atof(minVal.data());
			if (client.call(srv)) {

				if (srv.response.ack)
					ROS_INFO("Position Safety Bounds Set");
			} else {
				success = false;
			}

		} else if (safeName.compare("Position e") == 0) {

			beginner_tutorials::SAFE_POS_E srv;
			srvName << "/joint_" << i << "_set_safe_Pos_E";

			client = n.serviceClient<beginner_tutorials::SAFE_POS_E>(
					srvName.str());

			srv.request.SPEM = atof(maxVal.data());
			srv.request.SPEm = atof(minVal.data());
			if (client.call(srv)) {

				if (srv.response.ack)
					ROS_INFO("Position Error Safety Bounds Set");
			} else {
				success = false;
			}

		} else if (safeName.compare("Speed") == 0) {

			beginner_tutorials::SAFE_SPEED srv;
			srvName << "/joint_" << i << "_set_safe_Speed";

			client = n.serviceClient<beginner_tutorials::SAFE_SPEED>(
					srvName.str());

			srv.request.maxS = atof(maxVal.data());
			srv.request.minS = atof(minVal.data());
			if (client.call(srv)) {

				if (srv.response.ack)
					ROS_INFO("Speed Safety Bounds Set");
			} else {
				success = false;
			}

		} else if (safeName.compare("Speed e") == 0) {

			beginner_tutorials::SAFE_SPEED srv;
			srvName << "/joint_" << i << "_set_safe_Speed_E";

			client = n.serviceClient<beginner_tutorials::SAFE_SPEED>(
					srvName.str());

			srv.request.maxS = atof(maxVal.data());
			srv.request.minS = atof(minVal.data());
			if (client.call(srv)) {

				if (srv.response.ack)
					ROS_INFO("Speed Error Safety Bounds Set");
			} else {
				success = false;
			}

		} else if (safeName.compare("PWM") == 0) {

			beginner_tutorials::SAFE_PWM srv;
			srvName << "/joint_" << i << "_set_safe_PWM";

			client = n.serviceClient<beginner_tutorials::SAFE_PWM>(
					srvName.str());

			srv.request.maxP = atof(maxVal.data());
			srv.request.minP = atof(minVal.data());
			if (client.call(srv)) {

				if (srv.response.ack)
					ROS_INFO("PWM Safety Bounds Set");
			} else {
				success = false;
			}

		} else if (safeName.compare("Current") == 0) {

			beginner_tutorials::SAFE_CURR srv;
			srvName << "/joint_" << i << "_set_safe_curr";

			client = n.serviceClient<beginner_tutorials::SAFE_CURR>(
					srvName.str());

			srv.request.maxC = atof(maxVal.data());
			srv.request.minC = atof(minVal.data());
			if (client.call(srv)) {

				if (srv.response.ack)
					ROS_INFO("Current Safety Bounds Set");
			} else {
				success = false;
			}

		} else if (safeName.compare("Current e") == 0) {

			//TODO: implement!!!
			ROS_INFO("Not yet implemented");
			success = false;

		} else if (safeName.compare("Force") == 0) {

			beginner_tutorials::SAFE_FORCE srv;
			srvName << "/joint_" << i << "_set_safe_force";

			client = n.serviceClient<beginner_tutorials::SAFE_FORCE>(
					srvName.str());

			srv.request.maxF = atof(maxVal.data());
			srv.request.minF = atof(minVal.data());
			if (client.call(srv)) {

				if (srv.response.ack)
					ROS_INFO("Force Safety Bounds Set");
			} else {
				success = false;
			}
		} else {
			success = false;
		}

		if (!success)

			ROS_ERROR("Failed to call service %s", srvName.str().data());
	}
}

void TmOperation::ctlParSelected(const QString& parName) {

	if (parName.contains("PID"))

		ui_.set_par_stackedWidget->setCurrentIndex(
				ui_.set_par_stackedWidget->indexOf(ui_.pid_page));

	else if (parName.contains("Interp"))

		ui_.set_par_stackedWidget->setCurrentIndex(
				ui_.set_par_stackedWidget->indexOf(ui_.break_int_page));

	else if (parName.contains("FS"))

		ui_.set_par_stackedWidget->setCurrentIndex(
				ui_.set_par_stackedWidget->indexOf(ui_.fs_page));

	else if (parName.contains("PWM"))

		ui_.set_par_stackedWidget->setCurrentIndex(
				ui_.set_par_stackedWidget->indexOf(ui_.pwm_pars_page));
}

void TmOperation::setCtlPars() {

	ros::NodeHandle n = getNodeHandle();
	ros::ServiceClient client;

	stringstream srvName;

	beginner_tutorials::SELECT_CTL selSrv;
	beginner_tutorials::SET_SYS_FS fsSrv;
	beginner_tutorials::SET_INTER interSrv;
	beginner_tutorials::SET_PAR_PID pidSrv;
	beginner_tutorials::set_pwm_pars pwmSrv;

	string parValue;

	int _jointNum = ui_.j_sel_comboBox->currentIndex();

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");

		//////////////////////////////////////////////////////////////////////////////////

		if (ui_.ctl_pars_comboBox->currentText().contains("FS")) {

			srvName << "/joint_" << i << "_set_sys_FS";

			client = n.serviceClient<beginner_tutorials::SET_SYS_FS>(
					srvName.str());

			parValue = ui_.fs_plainTextEdit->toPlainText().toStdString();

			if (!isFloat(parValue)) {

				ROS_ERROR("Fs must be float");
				return;
			}

			fsSrv.request.sysFS = atof(parValue.data());

			if (client.call(fsSrv)) {

				if (!fsSrv.response.ack)
					ROS_INFO("joint %d FS Set", i);
				else
					ROS_INFO("Unable To Set joint %d FS ", i);

			} else {
				ROS_ERROR("Failed to call service %s", srvName.str().data());
			}

			//////////////////////////////////////////////////////////////////////////////////////////////

		} else if (ui_.ctl_pars_comboBox->currentText().contains("Interp")) {

			srvName << "/joint_" << i << "_select_Ctl";

			client = n.serviceClient<beginner_tutorials::SELECT_CTL>(
					srvName.str());

			selSrv.request.id =
					beginner_tutorials::SELECT_CTL::Request::BREAK_CTL;

			if (client.call(selSrv)) {

				if (!selSrv.response.ack) {
					ROS_ERROR("Unable to  select Joint %d scheme", i);

					continue;
				}
			} else {

				ROS_ERROR("Failed to call service %s", srvName.str().data());

				continue;
			}

			srvName.str("");

			srvName << "/joint_" << i << "_set_inter";

			client = n.serviceClient<beginner_tutorials::SET_INTER>(
					srvName.str());

			parValue = ui_.t_change_plainTextEdit->toPlainText().toStdString();

			if (!isFloat(parValue)) {

				ROS_ERROR("t Change must be float");
				return;
			}

			interSrv.request.time_change = atof(parValue.data());

			if (ui_.int_fun_comboBox->currentText().contains("Linear"))
				interSrv.request.id =
						beginner_tutorials::SET_INTER::Request::FLIN;

			else if (ui_.int_fun_comboBox->currentText().contains("Arc"))
				interSrv.request.id =
						beginner_tutorials::SET_INTER::Request::FARC;

			if (client.call(interSrv)) {

				if (!interSrv.response.ack)
					ROS_INFO("joint %d Break interpolator configured", i);
				else
					ROS_INFO("Unable To configure joint %d interpolator ", i);

			} else {
				ROS_ERROR("Failed to call service %s", srvName.str().data());
			}

			///////////////////////////////////////////////////////////////////////////////////////////////

		} else if (ui_.ctl_pars_comboBox->currentText().contains("PWM")) {

			srvName << "/joint_" << i << "_set_PWM_pars";

			client = n.serviceClient<beginner_tutorials::set_pwm_pars>(
					srvName.str());

			if (ui_.db_pos_plainTextEdit->toPlainText().isEmpty()) {
				pwmSrv.request.deadBandPos = 0.1;
			} else {

				parValue =
						ui_.db_pos_plainTextEdit->toPlainText().toStdString();

				if (!isFloat(parValue)) {

					ROS_ERROR("Parameters must be float");
					return;
				}

				pwmSrv.request.deadBandPos = atof(parValue.data());
			}

			if (ui_.db_neg_plainTextEdit->toPlainText().isEmpty()) {
				pwmSrv.request.deadBandNeg = 0.1;
			} else {

				parValue =
						ui_.db_neg_plainTextEdit->toPlainText().toStdString();

				if (!isFloat(parValue)) {

					ROS_ERROR("Parameters must be float");
					return;
				}

				pwmSrv.request.deadBandNeg = atof(parValue.data());
			}

			if (ui_.slope_pos_plainTextEdit->toPlainText().isEmpty()) {
				pwmSrv.request.slopePos = 1;
			} else {

				parValue =
						ui_.slope_pos_plainTextEdit->toPlainText().toStdString();

				if (!isFloat(parValue)) {

					ROS_ERROR("Parameters must be float");
					return;
				}

				pwmSrv.request.slopePos = atof(parValue.data());
			}

			if (ui_.slope_neg_plainTextEdit->toPlainText().isEmpty()) {
				pwmSrv.request.slopeNeg = 1;
			} else {

				parValue =
						ui_.slope_neg_plainTextEdit->toPlainText().toStdString();

				if (!isFloat(parValue)) {

					ROS_ERROR("Parameters must be float");
					return;
				}

				pwmSrv.request.slopeNeg = atof(parValue.data());
			}

			if (client.call(pwmSrv)) {

				if (pwmSrv.response.ack)
					ROS_INFO("joint %d PWM Parameters set", i);
				else
					ROS_INFO("Unable To configure joint %d PWM Parameters ", i);

			} else {
				ROS_ERROR("Failed to call service %s", srvName.str().data());
			}

			/////////////////////////////////////////////////////////////////////////////////////////////////

		} else if (ui_.ctl_pars_comboBox->currentText().contains("PID")) {

			if (ui_.ctl_pars_comboBox->currentText().contains("Pos")) {

				pidSrv.request.pid_id =
						beginner_tutorials::SET_PAR_PID::Request::POS_PID;

				selSrv.request.id =
						beginner_tutorials::SELECT_CTL::Request::POS_CTL;
			} else if (ui_.ctl_pars_comboBox->currentText().contains("Speed")) {

				pidSrv.request.pid_id =
						beginner_tutorials::SET_PAR_PID::Request::SPEED_PID;

				selSrv.request.id =
						beginner_tutorials::SELECT_CTL::Request::SPEED_CTL;
			} else if (ui_.ctl_pars_comboBox->currentText().contains("Force")) {

				pidSrv.request.pid_id =
						beginner_tutorials::SET_PAR_PID::Request::FORCE_PID;

				selSrv.request.id =
						beginner_tutorials::SELECT_CTL::Request::FORCE_CTL;
			} else if (ui_.ctl_pars_comboBox->currentText().contains("Break")) {

				pidSrv.request.pid_id =
						beginner_tutorials::SET_PAR_PID::Request::BREAK_PID;

				selSrv.request.id =
						beginner_tutorials::SELECT_CTL::Request::BREAK_CTL;
			} else if (ui_.ctl_pars_comboBox->currentText().contains("Index")) {

				pidSrv.request.pid_id =
						beginner_tutorials::SET_PAR_PID::Request::IDX_PID;

				selSrv.request.id =
						beginner_tutorials::SELECT_CTL::Request::IDX_CTL;

			}

			srvName << "/joint_" << i << "_select_Ctl";

			client = n.serviceClient<beginner_tutorials::SELECT_CTL>(
					srvName.str());

			if (client.call(selSrv)) {

				if (!selSrv.response.ack) {
					ROS_ERROR("Unable to  select Joint %d scheme", i);

					continue;
				}
			} else {

				ROS_ERROR("Failed to call service %s", srvName.str().data());

				continue;
			}

			srvName.str("");

			srvName << "/joint_" << i << "_set_par_pid";

			client = n.serviceClient<beginner_tutorials::SET_PAR_PID>(
					srvName.str());

			if (!ui_.kp_plainTextEdit->toPlainText().isEmpty()) {

				pidSrv.request.par_id =
						beginner_tutorials::SET_PAR_PID::Request::KP;

				parValue = ui_.kp_plainTextEdit->toPlainText().toStdString();

				if (isFloat(parValue)) {

					pidSrv.request.value = atof(parValue.data());

					if (client.call(pidSrv)) {

						if (!fsSrv.response.ack)
							ROS_INFO("joint %d %s' KP set", i,
									ui_.ctl_pars_comboBox->currentText().toStdString().data());
						else
							ROS_INFO("Unable To configure joint %d %s' KP", i,
									ui_.ctl_pars_comboBox->currentText().toStdString().data());

					} else {
						ROS_ERROR("Failed to call service %s",
								srvName.str().data());
					}
				} else {

					ROS_ERROR("KP must be float");
				}

			}

			if (!ui_.ki_plainTextEdit->toPlainText().isEmpty()) {

				pidSrv.request.par_id =
						beginner_tutorials::SET_PAR_PID::Request::KI;

				parValue = ui_.ki_plainTextEdit->toPlainText().toStdString();

				if (isFloat(parValue)) {

					pidSrv.request.value = atof(parValue.data());

					if (client.call(pidSrv)) {

						if (!fsSrv.response.ack)
							ROS_INFO("joint %d %s' KI set", i,
									ui_.ctl_pars_comboBox->currentText().toStdString().data());
						else
							ROS_INFO("Unable To configure joint %d %s' KI", i,
									ui_.ctl_pars_comboBox->currentText().toStdString().data());

					} else {
						ROS_ERROR("Failed to call service %s",
								srvName.str().data());
					}
				} else {

					ROS_ERROR("KI must be float");
				}

			}

			if (!ui_.kd_plainTextEdit->toPlainText().isEmpty()) {

				pidSrv.request.par_id =
						beginner_tutorials::SET_PAR_PID::Request::KD;

				parValue = ui_.kd_plainTextEdit->toPlainText().toStdString();

				if (isFloat(parValue)) {

					pidSrv.request.value = atof(parValue.data());

					if (client.call(pidSrv)) {

						if (!fsSrv.response.ack)
							ROS_INFO("joint %d %s' KD set", i,
									ui_.ctl_pars_comboBox->currentText().toStdString().data());
						else
							ROS_INFO("Unable To configure joint %d %s' KD", i,
									ui_.ctl_pars_comboBox->currentText().toStdString().data());

					} else {
						ROS_ERROR("Failed to call service %s",
								srvName.str().data());
					}
				} else {

					ROS_ERROR("KD must be float");
				}

			}

			if (!ui_.isat_plainTextEdit->toPlainText().isEmpty()) {

				pidSrv.request.par_id =
						beginner_tutorials::SET_PAR_PID::Request::ISAT;

				parValue = ui_.isat_plainTextEdit->toPlainText().toStdString();

				if (isFloat(parValue)) {

					pidSrv.request.value = atof(parValue.data());

					if (client.call(pidSrv)) {

						if (!fsSrv.response.ack)
							ROS_INFO("joint %d %s' ISat set", i,
									ui_.ctl_pars_comboBox->currentText().toStdString().data());
						else
							ROS_INFO("Unable To configure joint %d %s' ISat", i,
									ui_.ctl_pars_comboBox->currentText().toStdString().data());

					} else {
						ROS_ERROR("Failed to call service %s",
								srvName.str().data());
					}
				} else {

					ROS_ERROR("ISat must be float");
				}

			}

			if (!ui_.sat_plainTextEdit->toPlainText().isEmpty()) {

				pidSrv.request.par_id =
						beginner_tutorials::SET_PAR_PID::Request::SAT;

				parValue = ui_.sat_plainTextEdit->toPlainText().toStdString();

				if (isFloat(parValue)) {

					pidSrv.request.value = atof(parValue.data());

					if (client.call(pidSrv)) {

						if (!fsSrv.response.ack)
							ROS_INFO("joint %d %s' Sat set", i,
									ui_.ctl_pars_comboBox->currentText().toStdString().data());
						else
							ROS_INFO("Unable To configure joint %d %s' Sat", i,
									ui_.ctl_pars_comboBox->currentText().toStdString().data());

					} else {
						ROS_ERROR("Failed to call service %s",
								srvName.str().data());
					}
				} else {

					ROS_ERROR("Sat must be float");
				}

			}

		}

	}
}

void TmOperation::EnableIdx(int index) {

	if (index)
		ui_.start_idx_pushButton->setEnabled(true);
	else
		ui_.start_idx_pushButton->setEnabled(false);
}

void TmOperation::toggelIndex() {

	ros::NodeHandle n = getNodeHandle();
	ros::ServiceClient client;

	stringstream srvName;
	srvName.str("");

	int jNum = ui_.j_sel_comboBox->currentIndex();

	if (!jointIdxRunning_) {

		switch (jNum) {
		case 1:

			if (!ui_.j1_stat_textBrowser->toPlainText().contains(
					"Not Indexed")) {

				ROS_INFO("Cannot run joint %d indexing now", jNum);

				return;
			}

			break;
		case 2:

			if (!ui_.j2_stat_textBrowser->toPlainText().contains(
					"Not Indexed")) {

				ROS_INFO("Cannot run joint %d indexing now", jNum);

				return;
			}
			break;
		case 3:

			if (!ui_.j3_stat_textBrowser->toPlainText().contains(
					"Not Indexed")) {

				ROS_INFO("Cannot run joint %d indexing now", jNum);

				return;
			}
			break;
		default:

			return;
			break;
		}

		srvName.str("");
		srvName << "/joint_" << jNum << "_start";

		client = n.serviceClient<beginner_tutorials::START>(srvName.str());

		beginner_tutorials::START startSrv;

		if (client.call(startSrv)) {

			if (startSrv.response.ack) {
				ROS_INFO("Joint %d indexing started", jNum);
				jointIdxRunning_ = jNum;

				ui_.start_idx_pushButton->setText("Stop Indexing");
			}
		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());
		}

	} else {

		srvName << "/joint_" << jNum << "_stop";

		client.shutdown();

		client = n.serviceClient<beginner_tutorials::STOP>(srvName.str());

		beginner_tutorials::STOP stopSrv;

		if (client.call(stopSrv)) {

			if (stopSrv.response.ack)
				ROS_INFO("Joint %d indexing Stopped", jointIdxRunning_);

		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

		}

		jointIdxRunning_ = 0;

		ui_.start_idx_pushButton->setText("Start Indexing");
	}

}

void TmOperation::toggelIndexPosRef() {

	if (!jointIdxRunning_)
		return;

	ros::NodeHandle n = getNodeHandle();
	stringstream pubName;

	pubName.clear();
	pubName << "/joint_" << jointIdxRunning_ << "_pos_ref";

	posRefPub_ = n.advertise<beginner_tutorials::POS_REF>(pubName.str(), 1000);

	beginner_tutorials::POS_REF refMsg;

	if (ui_.p_idx_ref_pushButton->isDown())
		refMsg.q_D = 1;
	else
		refMsg.q_D = 0;

	posRefPub_.publish(refMsg);

}

void TmOperation::toggelIndexNegRef() {

	if (!jointIdxRunning_)
		return;

	ros::NodeHandle n = getNodeHandle();
	stringstream pubName;

	pubName.clear();
	pubName << "/joint_" << jointIdxRunning_ << "_pos_ref";

	posRefPub_ = n.advertise<beginner_tutorials::POS_REF>(pubName.str(), 1000);

	beginner_tutorials::POS_REF refMsg;

	if (ui_.n_idx_ref_pushButton->isDown())
		refMsg.q_D = -1;
	else
		refMsg.q_D = 0;

	posRefPub_.publish(refMsg);
}

void TmOperation::toggelOperation() {

	ros::NodeHandle n = getNodeHandle();
	ros::ServiceClient client;

	stringstream srvName;
	srvName.str("");

	if (!sysRunning_) {

		if (!ui_.sys_status_textBrowser->toPlainText().contains("Sys Ready")) {

			ROS_INFO("Cannot Start System, Sys not ready");

			return;
		}

		selOpMode();

		if (!jntCtlSelected_)
			return;

		srvName << "/start_ctl";

		client = n.serviceClient<beginner_tutorials::START_TM_CTL>(
				srvName.str());

		beginner_tutorials::START_TM_CTL startSrv;

		if (client.call(startSrv)) {

			if (startSrv.response.ack) {
				ROS_INFO("Sys started");
				ui_.start_op_pushButton->setText("Stop Operation");
				ui_.sel_op_mode_pushButton->setEnabled(false);
				ui_.g_comp_checkBox->setEnabled(false);
			}
		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());
		}

	} else {

		srvName << "/stop_ctl";

		client = n.serviceClient<beginner_tutorials::STOP_TM_CTL>(
				srvName.str());

		beginner_tutorials::STOP_TM_CTL stopSrv;

		if (client.call(stopSrv)) {

			if (stopSrv.response.ack) {
				ROS_INFO("Sys Stopped");
			}
		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

		}

		ui_.sel_op_mode_pushButton->setEnabled(true);
		ui_.g_comp_checkBox->setEnabled(true);
		ui_.start_op_pushButton->setText("Start Operation");
	}

}

void TmOperation::selOpMode() {

	ros::NodeHandle n = getNodeHandle();

	beginner_tutorials::SELECT_CTL selSrv;

	ros::ServiceClient client;

	QWidget * opModeWidget;

	beginner_tutorials::CTL_MODE srv;

	if (ui_.op_mode_comboBox->currentText().contains("PTP")) {

		selSrv.request.id = beginner_tutorials::SELECT_CTL::Request::POS_CTL;

		srv.request.mode = beginner_tutorials::CTL_MODE::Request::PTP;

		opModeWidget = ui_.ptp_page;
	} else if (ui_.op_mode_comboBox->currentText().contains("Force - Pos")) {

		selSrv.request.id = beginner_tutorials::SELECT_CTL::Request::POS_CTL;

		srv.request.mode = beginner_tutorials::CTL_MODE::Request::FORCE_POS;
		srv.request.force_k = 0.1;

		opModeWidget = ui_.f_pos_page;
	} else if (ui_.op_mode_comboBox->currentText().contains("Joint Traj")) {

		selSrv.request.id = beginner_tutorials::SELECT_CTL::Request::SPEED_CTL;

		opModeWidget = ui_.jnt_traj_page;
	}

	stringstream srvName;

	for (int i = 1; i < 4; ++i) {

		srvName.str("");

		srvName << "/joint_" << i << "_select_Ctl";

		client = n.serviceClient<beginner_tutorials::SELECT_CTL>(srvName.str());

		if (client.call(selSrv)) {

			if (!selSrv.response.ack) {
				ROS_ERROR("Unable to  select Joint %d scheme", i);

				continue;
			}
		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

			jntCtlSelected_ = false;

			return;
		}
	}

	jntCtlSelected_ = true;

	client = n.serviceClient<beginner_tutorials::CTL_MODE>("/tm_ctl_set_mode");

	if (client.call(srv)) {

		if (srv.response.ack) {
			ROS_INFO("Ctl mode set");

			ui_.op_panel_stackedWidget->setCurrentWidget(opModeWidget);
		}
	} else {

		ROS_ERROR("Failed to call tm_ctl_set_mode");
	}
}

void TmOperation::ptpModLabels(const QString & ptpMode) {

	if (ptpMode.contains("Cart")) {
		ui_.ptp_par1_label->setText("X des");
		ui_.ptp_par2_label->setText("Y des");
		ui_.ptp_par3_label->setText("Z des");
	} else if (ptpMode.contains("Joint")) {
		ui_.ptp_par1_label->setText("q1 des");
		ui_.ptp_par2_label->setText("q2 des");
		ui_.ptp_par3_label->setText("q3 des");
	}

}

void TmOperation::goHome() {

	ros::NodeHandle n = getNodeHandle();
	ros::ServiceClient client = n.serviceClient<beginner_tutorials::PTP_TM_CTL>(
			"/tm_ptp");

	beginner_tutorials::PTP_TM_CTL srv;

	srv.request.mode = beginner_tutorials::PTP_TM_CTL::Request::PTP_JNT;

	srv.request.Xd = 0;
	srv.request.Yd = 1.57;
	srv.request.Zd = 0;
	if (client.call(srv)) {

		if (srv.response.ack) {
			ROS_INFO("PTP set");
		}

	} else {

		ROS_ERROR("Failed to call");
	}
}

void TmOperation::ptpExe() {

	std::string xDes = ui_.x_des_plainTextEdit->toPlainText().toStdString();
	std::string yDes = ui_.y_des_plainTextEdit->toPlainText().toStdString();
	std::string zDes = ui_.z_des_plainTextEdit->toPlainText().toStdString();

	if (!isFloat(xDes) || !isFloat(yDes) || !isFloat(zDes)) {

		ROS_ERROR("Arguments must be floats");

		return;
	}

	ros::NodeHandle n = getNodeHandle();
	ros::ServiceClient client = n.serviceClient<beginner_tutorials::PTP_TM_CTL>(
			"/tm_ptp");

	beginner_tutorials::PTP_TM_CTL srv;

	if (ui_.ptp_mode_comboBox->currentText().contains("Cart"))
		srv.request.mode = beginner_tutorials::PTP_TM_CTL::Request::PTP_CART;
	else if (ui_.ptp_mode_comboBox->currentText().contains("Joint"))
		srv.request.mode = beginner_tutorials::PTP_TM_CTL::Request::PTP_JNT;

	srv.request.Xd = atof(xDes.data());
	srv.request.Yd = atof(yDes.data());
	srv.request.Zd = atof(zDes.data());
	if (client.call(srv)) {

		if (srv.response.ack) {
			ROS_INFO("PTP set");
		}

	} else {

		ROS_ERROR("Failed to call");
	}
}

void TmOperation::updateSensK() {

	std::string fK = ui_.sens_k_plainTextEdit->toPlainText().toStdString();

	if (!isFloat(fK)) {

		ROS_ERROR("Argument must be floats service /tm_ptp");

		return;
	}

	ros::NodeHandle n = getNodeHandle();
	ros::ServiceClient client = n.serviceClient<beginner_tutorials::set_par>(
			"/tm_set_f_pos_kp");

	beginner_tutorials::set_par srv;

	srv.request.parVal = atof(fK.data());

	if (client.call(srv)) {

		if (srv.response.ack) {
			ROS_INFO("F - Pos Sensibility set");
		}

	} else {

		ROS_ERROR("Failed to call F - Pos Sensibility Set service");
	}

}

void TmOperation::updateScaleK() {

	std::string fK = ui_.scale_k_plainTextEdit->toPlainText().toStdString();

	if (!isFloat(fK)) {

		ROS_ERROR("Argument must be floats service /tm_ptp");

		return;
	}

	ros::NodeHandle n = getNodeHandle();
	ros::ServiceClient client = n.serviceClient<beginner_tutorials::set_par>(
			"/tm_set_scaling_k");

	beginner_tutorials::set_par srv;

	srv.request.parVal = atof(fK.data());

	if (client.call(srv)) {

		if (srv.response.ack) {
			ROS_INFO("Scale set");
		}

	} else {

		ROS_ERROR("Failed to call scale Set service");
	}

}

void TmOperation::callbackSysStatus(const beginner_tutorials::SYS_STATUS& msg) {

	processStatusUpdate(msg); // Send signal to sysStatusUpdated SLOT
}

void TmOperation::sysStatusUpdated(const beginner_tutorials::SYS_STATUS& msg) {

	processSysStatus(ui_.sys_status_textBrowser, msg);

	ui_.sys_status_op_textBrowser->setText(
			ui_.sys_status_textBrowser->toPlainText());
	ui_.sys_status_op_textBrowser->setTextColor(
			ui_.sys_status_textBrowser->textColor());

	processJointStatus(ui_.j1_stat_textBrowser, msg.j1_stat);
	processJointStatus(ui_.j2_stat_textBrowser, msg.j2_stat);
	processJointStatus(ui_.j3_stat_textBrowser, msg.j3_stat);

	if (!sysRunning_
			&& msg.sys_status == beginner_tutorials::SYS_STATUS::RUNNING)
		sysRunning_ = true;

	if (sysRunning_
			&& msg.sys_status != beginner_tutorials::SYS_STATUS::RUNNING) {

		toggelOperation();
		sysRunning_ = false;
	}

	switch (jointIdxRunning_) {
	case 1:

		if (msg.j1_stat.index)
			toggelIndex();

		break;
	case 2:

		if (msg.j2_stat.index)
			toggelIndex();

		break;
	case 3:

		if (msg.j3_stat.index)
			toggelIndex();

		break;
	default:
		break;
	}
}

void TmOperation::jntTrajExe() {

	std::string dPos = ui_.traj_dpos_plainTextEdit->toPlainText().toStdString();
	std::string duration =
			ui_.traj_dur_plainTextEdit->toPlainText().toStdString();
	std::string accT = ui_.traj_acct_plainTextEdit->toPlainText().toStdString();

	std::string nStepos =
			ui_.traj_num_steps_plainTextEdit->toPlainText().toStdString();

	if (!isFloat(dPos) || !isFloat(duration) || !isFloat(accT)) {

		ROS_ERROR("Arguments must be floats");

		return;
	}

	ros::NodeHandle n = getNodeHandle();
	ros::ServiceClient client = n.serviceClient<beginner_tutorials::jnt_traj>(
			"/exe_jnt_traj");

	beginner_tutorials::jnt_traj srv;

	srv.request.nSteps = 1;

	if (ui_.traj_int_type_comboBox->currentText().contains("Tg"))
		srv.request.intType = beginner_tutorials::jnt_traj::Request::ARC_TG;

	else if (ui_.traj_int_type_comboBox->currentText().contains("Linear"))
		srv.request.intType = beginner_tutorials::jnt_traj::Request::LIN;

	if (ui_.traj_var_comboBox->currentText().contains("Speed"))
		srv.request.varId = beginner_tutorials::jnt_traj::Request::SPEED;

	else if (ui_.traj_var_comboBox->currentText().contains("PWM Steps")) {
		srv.request.varId = beginner_tutorials::jnt_traj::Request::PWM;

		if (!isFloat(dPos) || nStepos.empty()) {

			ROS_ERROR("Arguments must be floats");

			return;
		}

		srv.request.nSteps = atof(nStepos.data());

	} else if (ui_.traj_var_comboBox->currentText().contains("PWM"))
		srv.request.varId = beginner_tutorials::jnt_traj::Request::PWM;

	srv.request.jntNum = ui_.jnt_traj_comboBox->currentIndex() + 1;
	srv.request.dPos = atof(dPos.data());
	srv.request.duration = atof(duration.data());
	srv.request.accT = atof(accT.data());

	ROS_INFO("Commanding Joint Trajectory");

	if (client.call(srv)) {

		if (srv.response.ack) {
			ROS_INFO("Joint Trajectory complete");
		}

	} else {

		ROS_ERROR("Failed to command Joint Trajectory");
	}
}

void TmOperation::toggleGComp(int state) {

	ros::NodeHandle n = getNodeHandle();

	setGCompPub_ = n.advertise<std_msgs::Bool>("/set_g_comp", 100);

	std_msgs::Bool msg;

	if (ui_.g_comp_checkBox->isChecked())
		msg.data = true;
	else
		msg.data = false;

	setGCompPub_.publish(msg);
}

void TmOperation::toggleIdxPose(int state) {

	ros::NodeHandle n = getNodeHandle();

	setGCompPub_ = n.advertise<std_msgs::Bool>("/index_pose", 100);

	std_msgs::Bool msg;

	if (ui_.idx_pose_checkBox->isChecked())
		msg.data = true;
	else
		msg.data = false;

	setGCompPub_.publish(msg);
}

void TmOperation::trajVarSelected(const QString& varName) {

	ui_.traj_steps_label->setVisible(false);
	ui_.traj_num_steps_plainTextEdit->setVisible(false);

	ui_.traj_dur_label->setText("Duration [s]");

	if (varName.contains("Speed")) {

		ui_.traj_delta_label->setText("delta Pos [rad]");

	} else if (varName.contains("PWM Steps")) {

		ui_.traj_delta_label->setText("delta Pos [rad]");
		ui_.traj_dur_label->setText("PWM max [Nm]");
		ui_.traj_num_steps_plainTextEdit->setEnabled(true);

		ui_.traj_steps_label->setVisible(true);
		ui_.traj_num_steps_plainTextEdit->setVisible(true);
	} else if (varName.contains("PWM")) {

		ui_.traj_delta_label->setText("Tmax [Nm]");
		ui_.traj_num_steps_plainTextEdit->setEnabled(false);

	}
}

void TmOperation::processJointStatus(QTextBrowser * jointText,
		const beginner_tutorials::SYS_STATUS::_j1_stat_type & jointStatus) {

	QColor redC(255, 0, 0);
	QColor greenC(0, 255, 0);
	QColor blueC(0, 0, 255);
	QColor yellowC(255, 255, 0);

	switch (jointStatus.status) {
	case beginner_tutorials::TOPIC_STATUS::S_IDLE:

		jointText->setText("Idle");
		jointText->setTextColor(greenC);
		break;
	case beginner_tutorials::TOPIC_STATUS::S_OUT_OF_SYNCH:

		jointText->setText("Out of Synch");
		jointText->setTextColor(redC);
		break;
	case beginner_tutorials::TOPIC_STATUS::S_RUNNING:

		jointText->setText("Running");
		jointText->setTextColor(greenC);
		break;
	case beginner_tutorials::TOPIC_STATUS::S_SELECT_SCHEME:

		jointText->setText("Selecting Scheme");
		jointText->setTextColor(blueC);
		break;
	case beginner_tutorials::TOPIC_STATUS::S_SET_PAR:

		jointText->setText("Setting Par");
		jointText->setTextColor(blueC);
		break;
	case beginner_tutorials::TOPIC_STATUS::S_START_SCHEME:

		jointText->setText("Starting");
		jointText->setTextColor(blueC);
		break;
	case beginner_tutorials::TOPIC_STATUS::S_STOP_SCHEME:

		jointText->setText("Stopping");
		jointText->setTextColor(blueC);
		break;
	case beginner_tutorials::TOPIC_STATUS::S_UNINDEXED:

		jointText->setText("Not Indexed");
		jointText->setTextColor(redC);
		break;
	case beginner_tutorials::TOPIC_STATUS::S_UNSAFE:

		jointText->setText("UNSAFE");
		jointText->setTextColor(redC);
		break;
	case beginner_tutorials::TOPIC_STATUS::S_UPDATE_SYS:

		jointText->setText("Updating");
		jointText->setTextColor(blueC);
		break;
	default:
		break;
	}
}

void TmOperation::processSysStatus(QTextBrowser * sysText,
		const beginner_tutorials::SYS_STATUS & sysStatus) {

	QColor redC(255, 0, 0);
	QColor greenC(0, 255, 0);
	QColor blueC(0, 0, 255);
	QColor yellowC(255, 255, 0);

	switch (sysStatus.sys_status) {
	case beginner_tutorials::SYS_STATUS::JNTS_OUT_OF_SYNCH:

		sysText->setText("Not Synched");
		sysText->setTextColor(redC);
		break;
	case beginner_tutorials::SYS_STATUS::MONITOR_BREAK:

		sysText->setText("Breaking");
		sysText->setTextColor(yellowC);
		break;
	case beginner_tutorials::SYS_STATUS::RESTORE_SAFE:

		sysText->setText("Not Safe");
		sysText->setTextColor(redC);
		break;
	case beginner_tutorials::SYS_STATUS::RUNNING:

		sysText->setText("Running");
		sysText->setTextColor(greenC);
		break;
	case beginner_tutorials::SYS_STATUS::STOP_JOINTS:

		sysText->setText("Stopping");
		sysText->setTextColor(yellowC);
		break;
	case beginner_tutorials::SYS_STATUS::SYS_READY:

		sysText->setText("Sys Ready");
		sysText->setTextColor(greenC);
		break;
	case beginner_tutorials::SYS_STATUS::UNINDEXED:

		sysText->setText("Not Indexed");
		sysText->setTextColor(redC);
		break;
	default:
		break;
	}
}

bool isFloat(std::string myString) {
	istringstream iss(myString);
	float f;
	iss >> noskipws >> f; // noskipws considers leading whitespace invalid
	// Check the entire string was consumed and if either failbit or badbit is set
	return iss.eof() && !iss.fail();
}

}

PLUGINLIB_DECLARE_CLASS(rqt_tm_operation, TmOperation,
		rqt_tm_operation::TmOperation, rqt_gui_cpp::Plugin)
