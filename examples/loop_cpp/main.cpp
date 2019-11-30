#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

#include <rrosace.h>

using namespace RROSACE;

#define SIMPLE_LOOP_VZ_C (2.5)

class Simulation {
public:
  struct Values {
    FlightMode::Mode mode;
    double delta_e;
    double delta_e_c_partial_1;
    double delta_e_c_partial_2;
    double delta_th_c_partial_1;
    double delta_th_c_partial_2;
    RelayState relay_delta_e_c_1;
    RelayState relay_delta_e_c_2;
    RelayState relay_delta_th_c_1;
    RelayState relay_delta_th_c_2;
    double delta_e_c;
    double delta_th_c;
    double t;
    double h;
    double vz;
    double va;
    double q;
    double az;
    double h_f;
    double vz_f;
    double va_f;
    double q_f;
    double az_f;
    MasterInLaw master_in_law_1;
    MasterInLaw master_in_law_2;
    MasterInLaw other_master_in_law_1;
    MasterInLaw other_master_in_law_2;
    double h_c;
    double vz_c;
    double va_c;

    void print(double time) const {
      static const size_t t_prec = 3;
      static const size_t v_prec = 6;

      std::cout << std::fixed << std::setprecision(t_prec) << time << ",";
      std::cout << std::fixed << std::setprecision(v_prec) << h << ",";
      std::cout << std::fixed << std::setprecision(v_prec) << vz << ",";
      std::cout << std::fixed << std::setprecision(v_prec) << va << std::endl;
    }
  };

private:
  struct Models {

    Engine engine;
    Elevator elevator;
    FlightDynamics flight_dynamics;
    AltitudeFilter h_filter;
    VerticalAirspeedFilter vz_filter;
    TrueAirspeedFilter va_filter;
    PitchRateFilter q_filter;
    VerticalAccelerationFilter az_filter;
    FlightMode flight_mode;
    FlightControlUnit fcu;
    FlightControlComputer fcc1a;
    FlightControlComputer fcc1b;
    FlightControlComputer fcc2a;
    FlightControlComputer fcc2b;
    Cables cables;

    explicit Models(Values &values)
        : engine(TAU, values.delta_th_c, values.t),
          elevator(OMEGA, XI, values.delta_e_c, values.delta_e),
          flight_dynamics(values.delta_e, values.t, values.h, values.vz,
                          values.va, values.q, values.az),
          h_filter(values.h, values.h_f), vz_filter(values.vz, values.vz_f),
          va_filter(values.va, values.va_f), q_filter(values.q, values.q_f),
          az_filter(values.az, values.az_f),
          flight_mode(values.mode, values.mode),
          fcu(values.h_c, values.vz_c, values.va_c, values.h_c, values.vz_c,
              values.va_c),
          fcc1a(values.mode, values.h_f, values.vz_f, values.va_f, values.q_f,
                values.az_f, values.h_c, values.vz_c, values.va_c,
                values.delta_e_c_partial_1, values.delta_th_c_partial_1),
          fcc1b(values.mode, values.h_f, values.vz_f, values.va_f, values.q_f,
                values.az_f, values.h_c, values.vz_c, values.va_c,
                values.delta_e_c_partial_1, values.delta_th_c_partial_1,
                values.other_master_in_law_1, values.relay_delta_e_c_1,
                values.relay_delta_th_c_1, values.master_in_law_1),
          fcc2a(values.mode, values.h_f, values.vz_f, values.va_f, values.q_f,
                values.az_f, values.h_c, values.vz_c, values.va_c,
                values.delta_e_c_partial_2, values.delta_th_c_partial_2),
          fcc2b(values.mode, values.h_f, values.vz_f, values.va_f, values.q_f,
                values.az_f, values.h_c, values.vz_c, values.va_c,
                values.delta_e_c_partial_2, values.delta_th_c_partial_2,
                values.other_master_in_law_2, values.relay_delta_e_c_2,
                values.relay_delta_th_c_2, values.master_in_law_2),
          cables(values.delta_e_c_partial_1, values.delta_th_c_partial_1,
                 values.relay_delta_e_c_1, values.relay_delta_th_c_1,
                 values.delta_e_c_partial_2, values.delta_th_c_partial_2,
                 values.relay_delta_e_c_2, values.relay_delta_th_c_2,
                 values.delta_e_c, values.delta_th_c) {}

    std::vector<Model *> get_access_vector() {
      std::vector<Model *> access_vector;

      access_vector.push_back(&engine);
      access_vector.push_back(&elevator);
      access_vector.push_back(&flight_dynamics);
      access_vector.push_back(&h_filter);
      access_vector.push_back(&vz_filter);
      access_vector.push_back(&va_filter);
      access_vector.push_back(&q_filter);
      access_vector.push_back(&az_filter);
      access_vector.push_back(&flight_mode);
      access_vector.push_back(&fcu);
      access_vector.push_back(&fcc1a);
      access_vector.push_back(&fcc1b);
      access_vector.push_back(&fcc2a);
      access_vector.push_back(&fcc2b);
      access_vector.push_back(&cables);

      return access_vector;
    }
  };

  Values m_values;
  Models m_models;
  double m_time;
  size_t m_logical_time;
  double m_limit;

  void step();
  void print() const;

public:
  Simulation(Values values, double limit);
  void loop();
};

Simulation::Simulation(Values values, double limit)
    : m_values(values), m_models(m_values), m_time(0.), m_logical_time(0),
      m_limit(limit) {}

void Simulation::loop() {
  for (m_time = 0.; m_time < m_limit;) {
    step();
    print();
  }
}

void Simulation::step() {
  static std::vector<Model *> models_access_vector =
      m_models.get_access_vector();

  for (std::vector<Model *>::iterator model_access_it =
           models_access_vector.begin();
       model_access_it != models_access_vector.end(); ++model_access_it) {
    Model *const model_access = *model_access_it;
    const size_t model_period =
        static_cast<size_t>(DEFAULT_PHYSICAL_FREQ * model_access->get_dt());
    if (m_logical_time % model_period == 0) {
      model_access->step();
    }
  }

  m_time = (++m_logical_time) * (1. / DEFAULT_PHYSICAL_FREQ);
}

void Simulation::print() const { m_values.print(m_time); }

int main() {
  const double limit = 50.0;

  const Simulation::Values values = {RROSACE_COMMANDED,
                                     DELTA_E_EQ,
                                     DELTA_E_C_EQ,
                                     DELTA_E_C_EQ,
                                     DELTA_TH_C_EQ,
                                     DELTA_TH_C_EQ,
                                     RROSACE_RELAY_CLOSED,
                                     RROSACE_RELAY_OPENED,
                                     RROSACE_RELAY_CLOSED,
                                     RROSACE_RELAY_OPENED,
                                     DELTA_E_C_EQ,
                                     DELTA_TH_C_EQ,
                                     T_EQ,
                                     H_EQ,
                                     VZ_EQ,
                                     VA_EQ,
                                     Q_EQ,
                                     AZ_EQ,
                                     H_F_EQ,
                                     VZ_F_EQ,
                                     VA_F_EQ,
                                     Q_F_EQ,
                                     AZ_F_EQ,
                                     RROSACE_MASTER_IN_LAW,
                                     RROSACE_NOT_MASTER_IN_LAW,
                                     RROSACE_NOT_MASTER_IN_LAW,
                                     RROSACE_MASTER_IN_LAW,
                                     H_EQ,
                                     SIMPLE_LOOP_VZ_C,
                                     VA_EQ};

  std::cout << "time (s),altitude (m),vertical speed (m/s),airspeed (m/s)"
            << std::endl;

  Simulation(values, limit).loop();

  return (EXIT_SUCCESS);
}
