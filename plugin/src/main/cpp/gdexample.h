#ifndef GDEXAMPLE_H
#define GDEXAMPLE_H

#include <godot_cpp/classes/node3d.hpp>

namespace godot {

class GDExample : public Node3D {
    GDCLASS(GDExample, Node3D)

private:
    double time_passed;
    double amplitude;
    double speed;

protected:
    static void _bind_methods();

public:
    GDExample();
    ~GDExample();

    void _process(double delta);

    void set_amplitude(const double p_amplitude);
    double get_amplitude() const;

    void set_speed(const double p_speed);
    double get_speed() const;
};

}

#endif