#include "sphere.h"

#include <Eigen/Dense>

#include "cloth.h"
#include "configs.h"

namespace {
void generateVertices(std::vector<GLfloat>& vertices, std::vector<GLuint>& indices) {
  // See http://www.songho.ca/opengl/gl_sphere.html#sphere if you don't know how to create a sphere.
  vertices.reserve(8 * (sphereStack + 1) * (sphereSlice + 1));
  indices.reserve(6 * sphereStack * sphereSlice);

  float x, y, z, xy;  //  position

  float sectorStep = static_cast<float>(EIGEN_PI * 2 / sphereSlice);
  float stackStep = static_cast<float>(EIGEN_PI / sphereStack);
  float sectorAngle, stackAngle;

  for (int i = 0; i <= sphereStack; ++i) {
    stackAngle = static_cast<float>(EIGEN_PI / 2 - i * stackStep);  // [pi/2, -pi/2]
    xy = cosf(stackAngle);                                          // r * cos(u)
    z = sinf(stackAngle);                                           // r * sin(u)

    for (int j = 0; j <= sphereSlice; ++j) {
      sectorAngle = j * sectorStep;  // [0, 2pi]

      x = xy * cosf(sectorAngle);  // r * cos(u) * cos(v)
      y = xy * sinf(sectorAngle);  // r * cos(u) * sin(v)
      vertices.insert(vertices.end(), {x, y, z, x, y, z});
    }
  }

  unsigned int k1, k2;  // EBO index
  for (int i = 0; i < sphereStack; ++i) {
    k1 = i * (sphereSlice + 1);  // beginning of current sphereStack
    k2 = k1 + sphereSlice + 1;   // beginning of next sphereStack
    for (int j = 0; j < sphereSlice; ++j, ++k1, ++k2) {
      if (i != 0) {
        indices.insert(indices.end(), {k1, k2, k1 + 1});
      }
      // k1+1 => k2 => k2+1
      if (i != (sphereStack - 1)) {
        indices.insert(indices.end(), {k1 + 1, k2, k2 + 1});
      }
    }
  }
}
}  // namespace

Spheres& Spheres::initSpheres() {
  static Spheres spheres;
  return spheres;
}

void Spheres::addSphere(const Eigen::Ref<const Eigen::Vector4f>& position, float size) {
  if (sphereCount == _particles.getCapacity()) {
    _particles.resize(sphereCount * 2);
    _radius.resize(sphereCount * 2);
    offsets.allocate(8 * sphereCount * sizeof(float));
    sizes.allocate(2 * sphereCount * sizeof(float));
  }
  _radius[sphereCount] = size;
  _particles.position(sphereCount) = position;
  _particles.velocity(sphereCount).setZero();
  _particles.acceleration(sphereCount).setZero();
  _particles.mass(sphereCount) = sphereDensity * size * size * size;

  sizes.load(0, _radius.size() * sizeof(float), _radius.data());
  ++sphereCount;
}

Spheres::Spheres() : Shape(1, 1), sphereCount(0), _radius(1, 0.0f) {
  offsets.allocate(4 * sizeof(float));
  sizes.allocate(sizeof(float));

  std::vector<GLfloat> vertices;
  std::vector<GLuint> indices;
  generateVertices(vertices, indices);

  vbo.allocate_load(vertices.size() * sizeof(GLfloat), vertices.data());
  ebo.allocate_load(indices.size() * sizeof(GLuint), indices.data());

  vao.bind();
  vbo.bind();
  ebo.bind();

  vao.enable(0);
  vao.setAttributePointer(0, 3, 6, 0);
  glVertexAttribDivisor(0, 0);
  vao.enable(1);
  vao.setAttributePointer(1, 3, 6, 3);
  glVertexAttribDivisor(1, 0);
  offsets.bind();
  vao.enable(2);
  vao.setAttributePointer(2, 3, 4, 0);
  glVertexAttribDivisor(2, 1);
  sizes.bind();
  vao.enable(3);
  vao.setAttributePointer(3, 1, 1, 0);
  glVertexAttribDivisor(3, 1);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Spheres::draw() const {
  vao.bind();
  offsets.load(0, 4 * sphereCount * sizeof(GLfloat), _particles.getPositionData());
  GLsizei indexCount = static_cast<GLsizei>(ebo.size() / sizeof(GLuint));
  glDrawElementsInstanced(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr, sphereCount);
  glBindVertexArray(0);
}

void Spheres::collide(Shape* shape) { shape->collide(this); }
void Spheres::collide(Cloth* cloth) {
  constexpr float coefRestitution = 0.0f;
  // TODO: Collide with particle (Simple approach to handle softbody collision)
  //   1. Detect collision.
  //   2. If collided, update impulse directly to particles' velocity
  //   3. (Bonus) You can add friction, which updates particles' acceleration a = F / m
  // Note:
  //   1. There are `sphereCount` spheres.
  //   2. There are `particlesPerEdge * particlesPerEdge` particles.
  //   3. See TODOs in Cloth::computeSpringForce if you don't know how to access data.
  // Hint:
  //   1. You can simply push particles back to prevent penetration.
  //     Sample code is provided here:
  //       Eigen::Vector4f correction = penetration * normal * 0.15f;
  //       _particles.position(i) += correction;
  //       _particles.position(j) -= correction;

  // Write code here!
  for (int i = 0; i < sphereCount; ++i) {
    auto mass_a = _particles.mass(i);
    for (int j = 0; j < particlesPerEdge * particlesPerEdge; ++j) {
      Eigen::Vector4f ori_velocity_a = _particles.velocity(i);

      if (this->radius(i) > (_particles.position(i) - cloth->particles().position(j)).norm()) { // detect collision
        Eigen::Vector4f normal_vector = (_particles.position(i) - cloth->particles().position(j)) / (_particles.position(i) - cloth->particles().position(j)).norm(); // 兩球間位置的單位向量，j指向i (b指向a)
        auto mass_b = cloth->particles().mass(j);
        Eigen::Vector4f ori_velocity_b = cloth->particles().velocity(j);

        /// 記得碰撞時只要計算normal方向的速度即可，切線方向的速度不會變
        Eigen::Vector4f ori_normal_velocity_a = ori_velocity_a.dot(normal_vector) * normal_vector;
        Eigen::Vector4f ori_tangent_velocity_a = ori_velocity_a - ori_normal_velocity_a;
        Eigen::Vector4f ori_normal_velocity_b = ori_velocity_b.dot(normal_vector) * normal_vector;
        Eigen::Vector4f ori_tangent_velocity_b = ori_velocity_b - ori_normal_velocity_b;

        if ((ori_normal_velocity_a - ori_normal_velocity_b).dot(normal_vector) < 0) // 若兩粒子有互相靠近，b指向a的速度和b指向a的距離向量內積小於零
        {
            Eigen::Vector4f normal_velocity_after_collision_a =
                (mass_a * ori_normal_velocity_a + mass_b * ori_normal_velocity_b +
                 mass_b * coefRestitution * (ori_normal_velocity_b - ori_normal_velocity_a)) /
                (mass_a + mass_b);

            _particles.velocity(i) = ori_tangent_velocity_a + normal_velocity_after_collision_a;

            Eigen::Vector4f normal_velocity_after_collision_b =
                (mass_a * ori_normal_velocity_a + mass_b * ori_normal_velocity_b +
                 mass_a * coefRestitution * (ori_normal_velocity_a - ori_normal_velocity_b)) /
                (mass_a + mass_b);

            cloth->particles().velocity(j) = ori_tangent_velocity_b + normal_velocity_after_collision_b;
        }

        /// 穿刺的correction，把他推回邊界
        auto penetration = this->radius(i) - (_particles.position(i) - cloth->particles().position(j)).norm();  // 穿刺進去的量
        Eigen::Vector4f correction = penetration * normal_vector * 0.5f;  //0.5是隨便取一個合理的值
        _particles.position(i) += correction;
        cloth->particles().position(j) -= correction;
      }
    }
  }
  /// end of my code
}

void Spheres::collide() {
  constexpr float coefRestitution = 0.8f;
  // TODO: Collide with another sphere (Rigidbody collision)
  //   1. Detect collision.
  //   2. If collided, update impulse directly to particles' velocity
  //   3. (Bonus) You can add friction, which updates particles' acceleration a = F / m
  // Note:
  //   1. There are `sphereCount` spheres.
  //   2. You may not want to calculate one sphere twice (a <-> b and b <-> a)
  //   3. See TODOs in Cloth::computeSpringForce if you don't know how to access data.
  // Hint:
  //   1. You can simply push particles back to prevent penetration.

  // Write code here!
  ///幾乎都和上面那個一樣
  for (int i = 0; i < sphereCount; ++i) {
    auto mass_a = _particles.mass(i);
    
    for (int j = i + 1; j < sphereCount; ++j) {
      Eigen::Vector4f ori_velocity_a = _particles.velocity(i);

      if ((this->radius(i) + this->radius(j)) > (_particles.position(i) - _particles.position(j)).norm()) {
        Eigen::Vector4f normal_vector = (_particles.position(i) - _particles.position(j)) /
                             (_particles.position(i) - _particles.position(j)).norm();  // 兩球間位置的單位向量，j指向i (b指向a)
        auto mass_b = _particles.mass(j);
        Eigen::Vector4f ori_velocity_b = _particles.velocity(j);

        /// 記得碰撞時只要計算normal方向的速度即可，切線方向的速度不會變
        Eigen::Vector4f ori_normal_velocity_a = ori_velocity_a.dot(normal_vector) * normal_vector;
        Eigen::Vector4f ori_tangent_velocity_a = ori_velocity_a - ori_normal_velocity_a;
        Eigen::Vector4f ori_normal_velocity_b = ori_velocity_b.dot(normal_vector) * normal_vector;
        Eigen::Vector4f ori_tangent_velocity_b = ori_velocity_b - ori_normal_velocity_b;

        if ((ori_normal_velocity_a - ori_normal_velocity_b).dot(normal_vector) < 0) // 若兩粒子有互相靠近，b指向a的速度和b指向a的距離向量內積小於零
        {
            Eigen::Vector4f normal_velocity_after_collision_a =
                (mass_a * ori_normal_velocity_a + mass_b * ori_normal_velocity_b +
                    mass_b * coefRestitution * (ori_normal_velocity_b - ori_normal_velocity_a)) /
                (mass_a + mass_b);

            _particles.velocity(i) = ori_tangent_velocity_a + normal_velocity_after_collision_a;

            Eigen::Vector4f normal_velocity_after_collision_b =
                (mass_a * ori_normal_velocity_a + mass_b * ori_normal_velocity_b +
                    mass_a * coefRestitution * (ori_normal_velocity_a - ori_normal_velocity_b)) /
                (mass_a + mass_b);

            _particles.velocity(j) = ori_tangent_velocity_b + normal_velocity_after_collision_b;
        }

        /// 穿刺的correction，把他推回邊界
        auto penetration = (this->radius(i) + this->radius(j)) - (_particles.position(i) - _particles.position(j)).norm();  // 穿刺進去的量
        Eigen::Vector4f correction = penetration * normal_vector * 0.15f;  // 0.15是隨便取一個合理的值
        _particles.position(i) += correction;
        _particles.position(j) -= correction;
      }
    }
  }
  /// end of my code
}
