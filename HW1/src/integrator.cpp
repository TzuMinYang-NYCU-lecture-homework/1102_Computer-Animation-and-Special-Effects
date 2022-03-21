#include "integrator.h"

#include "configs.h"

void ExplicitEuler::integrate(const std::vector<Particles *> &particles, std::function<void(void)>) const {
  // TODO: Integrate velocity and acceleration
  //   1. Integrate velocity.
  //   2. Integrate acceleration.
  //   3. You should not compute position using acceleration. Since some part only update velocity. (e.g. impulse)
  // Note:
  //   1. You don't need the simulation function in explicit euler.
  //   2. You should do this first because it is very simple. Then you can check your collision is correct or not.
  //   3. This can be done in 2 lines. (Hint: You can add / multiply all particles at once since it is a large matrix.)
  for (const auto &p : particles) {
    // Write code here!
    p->position() += deltaTime * p->velocity();
    p->velocity() += deltaTime * p->acceleration();
  }
}

void ImplicitEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  // Write code here!

  /// 不能用auto backup_particles = particles;來備份，因為裡面存的是指標
  /// 1.備份
  std::vector<Particles> backup_particles;
  for (const auto &p : particles) backup_particles.emplace_back(*p);

  /// 2.算速度
  for (const auto &p : particles) {
    p->position() += deltaTime * p->velocity();
    p->velocity() += deltaTime * p->acceleration();
  }
  simulateOneStep();

  /// 3.更新
  for (int i = 0; i < particles.size(); ++i) {
    particles[i]->position() = backup_particles[i].position() + deltaTime * particles[i]->velocity();
    particles[i]->velocity() = backup_particles[i].velocity() + deltaTime * particles[i]->acceleration();
    particles[i]->acceleration() = backup_particles[i].acceleration();
  }
  /// end of my code
}

void MidpointEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  // Write code here!
  
  /// 1.備份
  std::vector<Particles> backup_particles;
  for (const auto &p : particles) backup_particles.emplace_back(*p);
  
  /// 2.算速度
  for (const auto &p : particles) {
    p->position() += deltaTime * p->velocity() / 2;
    p->velocity() += deltaTime * p->acceleration() / 2;
  }
  simulateOneStep();

  /// 3.更新
  for (int i = 0; i < particles.size(); ++i) {
    particles[i]->position() = backup_particles[i].position() + deltaTime * particles[i]->velocity();
    particles[i]->velocity() = backup_particles[i].velocity() + deltaTime * particles[i]->acceleration();
    particles[i]->acceleration() = backup_particles[i].acceleration();
  }
  /// end of my code
}

void RungeKuttaFourth::integrate(const std::vector<Particles *> &particles,
                                 std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Compute k1, k2, k3, k4
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  // Write code here!

  /// 1.備份
  std::vector<Particles> backup_particles;
  for (const auto &p : particles) backup_particles.emplace_back(*p);

  /// 2.算k1 k2 k3 k4
  auto k1_particles = backup_particles;  //這樣寫只是為了方便
  for (int i = 0; i < particles.size(); ++i) {
    k1_particles[i].position() = deltaTime * particles[i]->velocity();
    k1_particles[i].velocity() = deltaTime * particles[i]->acceleration();
  }

  auto k2_particles = backup_particles;  //這樣寫只是為了方便
  for (int i = 0; i < particles.size(); ++i) {
    particles[i]->position() = backup_particles[i].position() + k1_particles[i].position() / 2;
    particles[i]->velocity() = backup_particles[i].velocity() + k1_particles[i].velocity() / 2;
  }
  simulateOneStep();
  for (int i = 0; i < particles.size(); ++i) {
    k2_particles[i].position() = deltaTime * particles[i]->velocity();
    k2_particles[i].velocity() = deltaTime * particles[i]->acceleration();
  }

  auto k3_particles = backup_particles;  //這樣寫只是為了方便
  for (int i = 0; i < particles.size(); ++i) {
    particles[i]->position() = backup_particles[i].position() + k2_particles[i].position() / 2;
    particles[i]->velocity() = backup_particles[i].velocity() + k2_particles[i].velocity() / 2;
    particles[i]->acceleration() = backup_particles[i].acceleration();
  }
  simulateOneStep();
  for (int i = 0; i < particles.size(); ++i) {
    k3_particles[i].position() = deltaTime * particles[i]->velocity();
    k3_particles[i].velocity() = deltaTime * particles[i]->acceleration();
  }

  auto k4_particles = backup_particles;  //這樣寫只是為了方便
  for (int i = 0; i < particles.size(); ++i) {
    particles[i]->position() = backup_particles[i].position() + k3_particles[i].position();
    particles[i]->velocity() = backup_particles[i].velocity() + k3_particles[i].velocity();
    particles[i]->acceleration() = backup_particles[i].acceleration();
  }
  simulateOneStep();
  for (int i = 0; i < particles.size(); ++i) {
    k4_particles[i].position() = deltaTime * particles[i]->velocity();
    k4_particles[i].velocity() = deltaTime * particles[i]->acceleration();
  }

  /// 3.更新
  for (int i = 0; i < particles.size(); ++i) {
    particles[i]->position() = backup_particles[i].position() + (k1_particles[i].position() + 2 * k2_particles[i].position() + 
                               2 * k3_particles[i].position() + k4_particles[i].position()) / 6;
    particles[i]->velocity() = backup_particles[i].velocity() + (k1_particles[i].velocity() + 2 * k2_particles[i].velocity() + 
                               2 * k3_particles[i].velocity() + k4_particles[i].velocity()) / 6;
    particles[i]->acceleration() = backup_particles[i].acceleration();
  }
  /// end of my code
}
