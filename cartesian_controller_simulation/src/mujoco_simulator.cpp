////////////////////////////////////////////////////////////////////////////////
// Copyright 2022 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    mujoco_simulator.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2022/02/01
 *
 */
//-----------------------------------------------------------------------------

#include "cartesian_controller_simulation/mujoco_simulator.h"

#include <iostream>
#include <memory>
#include <GLFW/glfw3.h>

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

namespace cartesian_controller_simulation
{
MuJoCoSimulator::MuJoCoSimulator() {}

void MuJoCoSimulator::controlCB(const mjModel * m, mjData * d)
{
  getInstance().controlCBImpl(m, d);
}

void MuJoCoSimulator::controlCBImpl([[maybe_unused]] const mjModel * m, mjData * d)
{
  command_mutex.lock();

  for (size_t i = 0; i < pos_cmd.size(); ++i)
  {
    // Joint-level impedance control
    d->ctrl[i] = stiff[i] * (pos_cmd[i] - d->qpos[i]) +             // stiffness
                 damp[i] * (vel_cmd[i] - d->actuator_velocity[i]);  // damping
  }
  command_mutex.unlock();
}

int MuJoCoSimulator::simulate(const std::string & model_xml, bool with_head)
{
  return with_head ? getInstance().simulateImplWHead(model_xml) : getInstance().simulateImpl(model_xml);
}

int MuJoCoSimulator::simulateImpl(const std::string & model_xml)
{
  // Make sure that the ROS2-control system_interface only gets valid data in read().
  // We lock until we are done with simulation setup.
  state_mutex.lock();

  // load and compile model
  char error[1000] = "Could not load binary model";
  m = mj_loadXML(model_xml.c_str(), nullptr, error, 1000);
  if (!m)
  {
    mju_error_s("Load model error: %s", error);
    return 1;
  }

  // Set initial state with the keyframe mechanism from xml
  d = mj_makeData(m);
  mju_copy(d->qpos, m->key_qpos, m->nq);

  // Initialize buffers for ROS2-control.
  pos_state.resize(m->nu);
  vel_state.resize(m->nu);
  eff_state.resize(m->nu);
  pos_cmd.resize(m->nu);
  vel_cmd.resize(m->nu);
  stiff.resize(m->nu);
  damp.resize(m->nu);

  // Start where we are
  syncStates();
  state_mutex.unlock();

  // Connect our specific control input callback for MuJoCo's engine.
  mjcb_control = MuJoCoSimulator::controlCB;

  // Simulate in realtime
  while (true)
  {
    mj_step(m, d);

    // Provide fresh data for ROS2-control
    state_mutex.lock();
    syncStates();
    state_mutex.unlock();
  }

  return 0;
}

int MuJoCoSimulator::simulateImplWHead(const std::string & model_xml){
  // Make sure that the ROS2-control system_interface only gets valid data in read().
  // We lock until we are done with simulation setup.
  state_mutex.lock();

  // load and compile model
  char error[1000] = "Could not load binary model";
  m = mj_loadXML(model_xml.c_str(), nullptr, error, 1000);
  if (!m)
  {
    mju_error_s("Load model error: %s", error);
    return 1;
  }

  // Set initial state with the keyframe mechanism from xml
  d = mj_makeData(m);
  mju_copy(d->qpos, m->key_qpos, m->nq);

  // Initialize buffers for ROS2-control.
  pos_state.resize(m->nu);
  vel_state.resize(m->nu);
  eff_state.resize(m->nu);
  pos_cmd.resize(m->nu);
  vel_cmd.resize(m->nu);
  stiff.resize(m->nu);
  damp.resize(m->nu);

  // Start where we are
  syncStates();
  state_mutex.unlock();

  // Connect our specific control input callback for MuJoCo's engine.
  mjcb_control = MuJoCoSimulator::controlCB;

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {
    // Simulate in realtime
    mjtNum simstart = d->time;
    // while (true)
    while (d->time - simstart < 1.0/60.0) {
      mj_step(m, d);

      // Provide fresh data for ROS2-control
      state_mutex.lock();
      syncStates();
      state_mutex.unlock();

    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

return 0;
}

void MuJoCoSimulator::read(std::vector<double> & pos, std::vector<double> & vel,
                           std::vector<double> & eff)
{
  // Realtime in ROS2-control is more important than fresh data exchange.
  if (state_mutex.try_lock())
  {
    pos = pos_state;
    vel = vel_state;
    eff = eff_state;
    state_mutex.unlock();
  }
}

void MuJoCoSimulator::write(const std::vector<double> & pos, const std::vector<double> & vel,
                            const std::vector<double> & stiff, const std::vector<double> & damp)
{
  // Realtime in ROS2-control is more important than fresh data exchange.
  if (command_mutex.try_lock())
  {
    pos_cmd = pos;
    vel_cmd = vel;
    this->stiff = stiff;
    this->damp = damp;
    command_mutex.unlock();
  }
}

void MuJoCoSimulator::syncStates()
{
  for (auto i = 0; i < m->nu; ++i)
  {
    pos_state[i] = d->qpos[i];
    vel_state[i] = d->actuator_velocity[i];
    eff_state[i] = d->actuator_force[i];
  }
}

}  // namespace cartesian_controller_simulation
