


//updated values for get_thrust_base_vector with -1 in the function

wrench_alloc.row(0) <<  k_f*std::pow(pwm_1,2)*(-0.5*std::sin(phi_1_1)*std::cos(phi_1_2) - 0.5*std::sin(phi_1_1) - 0.707106781186547*std::sin(phi_1_2)*std::cos(phi_1_1)) + k_f*std::pow(pwm_2,2)*(-0.209445028587455*std::sin(phi_2_1)*std::sin(phi_2_2) + 0.25*std::sin(phi_2_1)*std::cos(phi_2_2) + 0.25*std::sin(phi_2_1) + 0.353553390593274*std::sin(phi_2_2)*std::cos(phi_2_1) + 0.1481*std::cos(phi_2_1)*std::cos(phi_2_2) + 0.1481*std::cos(phi_2_1) + 0.4069*std::cos(phi_2_2) - 0.4069) + k_f*std::pow(pwm_3,2)*(0.209445028587455*std::sin(phi_3_1)*std::sin(phi_3_2) + 0.25*std::sin(phi_3_1)*std::cos(phi_3_2) + 0.25*std::sin(phi_3_1) + 0.353553390593274*std::sin(phi_3_2)*std::cos(phi_3_1) - 0.1481*std::cos(phi_3_1)*std::cos(phi_3_2) - 0.1481*std::cos(phi_3_1) - 0.4069*std::cos(phi_3_2) + 0.4069);
wrench_alloc.row(1) <<  k_f*std::pow(pwm_1,2)*(0.241830519165799*std::sin(phi_1_1)*std::sin(phi_1_2) - 0.171*std::cos(phi_1_1)*std::cos(phi_1_2) - 0.171*std::cos(phi_1_1) - 0.46985*std::cos(phi_1_2) + 0.46985) + k_f*std::pow(pwm_2,2)*(-0.1209152595829*std::sin(phi_2_1)*std::sin(phi_2_2) - 0.433*std::sin(phi_2_1)*std::cos(phi_2_2) - 0.433*std::sin(phi_2_1) - 0.61235447250755*std::sin(phi_2_2)*std::cos(phi_2_1) + 0.0855*std::cos(phi_2_1)*std::cos(phi_2_2) + 0.0855*std::cos(phi_2_1) + 0.2349*std::cos(phi_2_2) - 0.2349) + k_f*std::pow(pwm_3,2)*(-0.1209152595829*std::sin(phi_3_1)*std::sin(phi_3_2) + 0.433*std::sin(phi_3_1)*std::cos(phi_3_2) + 0.433*std::sin(phi_3_1) + 0.61235447250755*std::sin(phi_3_2)*std::cos(phi_3_1) + 0.0855*std::cos(phi_3_1)*std::cos(phi_3_2) + 0.0855*std::cos(phi_3_1) + 0.2349*std::cos(phi_3_2) - 0.2349);
wrench_alloc.row(2) <<  k_f*std::pow(pwm_1,2)*(-0.664468242280999*std::sin(phi_1_1)*std::sin(phi_1_2) + 0.46985*std::cos(phi_1_1)*std::cos(phi_1_2) + 0.46985*std::cos(phi_1_1) - 0.171*std::cos(phi_1_2) + 0.171) + k_f*std::pow(pwm_2,2)*(-0.664468242280999*std::sin(phi_2_1)*std::sin(phi_2_2) + 0.46985*std::cos(phi_2_1)*std::cos(phi_2_2) + 0.46985*std::cos(phi_2_1) - 0.171*std::cos(phi_2_2) + 0.171) + k_f*std::pow(pwm_3,2)*(-0.664468242280999*std::sin(phi_3_1)*std::sin(phi_3_2) + 0.46985*std::cos(phi_3_1)*std::cos(phi_3_2) + 0.46985*std::cos(phi_3_1) - 0.171*std::cos(phi_3_2) + 0.171);
wrench_alloc.row(3) <<  k_f*std::pow(pwm_1,2)*(-0.664468242280999*std::sin(phi_1_1)*std::sin(phi_1_2) + 0.46985*std::cos(phi_1_1)*std::cos(phi_1_2) + 0.46985*std::cos(phi_1_1) - 0.171*std::cos(phi_1_2) + 0.171)*(-0.00983663959984971*std::sin(phi_1_1)*std::sin(phi_1_2) + 0.00695555456514185*std::cos(phi_1_1)*std::cos(phi_1_2) + 0.0266288337648413*std::cos(phi_1_1) + 0.0191115047510637*std::cos(phi_1_2) - 0.229040012685816) + k_f*std::pow(pwm_1,2)*(-0.0270277492163122*std::sin(phi_1_1)*std::sin(phi_1_2) + 0.0191115047510637*std::cos(phi_1_1)*std::cos(phi_1_2) + 0.0731670031836881*std::cos(phi_1_1) - 0.00695555456514185*std::cos(phi_1_2) + 0.040539942895125)*(0.241830519165799*std::sin(phi_1_1)*std::sin(phi_1_2) - 0.171*std::cos(phi_1_1)*std::cos(phi_1_2) - 0.171*std::cos(phi_1_1) - 0.46985*std::cos(phi_1_2) + 0.46985) + k_f*std::pow(pwm_2,2)*(-0.664468242280999*std::sin(phi_2_1)*std::sin(phi_2_2) + 0.46985*std::cos(phi_2_1)*std::cos(phi_2_2) + 0.46985*std::cos(phi_2_1) - 0.171*std::cos(phi_2_2) + 0.171)*(0.00491831979992485*std::sin(phi_2_1)*std::sin(phi_2_2) + 0.0176126030801545*std::sin(phi_2_1)*std::cos(phi_2_2) + 0.0674285673694518*std::sin(phi_2_1) + 0.0249079821446487*std::sin(phi_2_2)*std::cos(phi_2_1) - 0.00347777728257093*std::cos(phi_2_1)*std::cos(phi_2_2) - 0.0133144168824206*std::cos(phi_2_1) - 0.0095547354815896*std::cos(phi_2_2) + 0.244639079450672) + k_f*std::pow(pwm_2,2)*(-0.0270277492163122*std::sin(phi_2_1)*std::sin(phi_2_2) + 0.0191115047510637*std::cos(phi_2_1)*std::cos(phi_2_2) + 0.0731670031836881*std::cos(phi_2_1) - 0.00695555456514185*std::cos(phi_2_2) + 0.040539942895125)*(-0.1209152595829*std::sin(phi_2_1)*std::sin(phi_2_2) - 0.433*std::sin(phi_2_1)*std::cos(phi_2_2) - 0.433*std::sin(phi_2_1) - 0.61235447250755*std::sin(phi_2_2)*std::cos(phi_2_1) + 0.0855*std::cos(phi_2_1)*std::cos(phi_2_2) + 0.0855*std::cos(phi_2_1) + 0.2349*std::cos(phi_2_2) - 0.2349) + k_f*std::pow(pwm_3,2)*(-0.664468242280999*std::sin(phi_3_1)*std::sin(phi_3_2) + 0.46985*std::cos(phi_3_1)*std::cos(phi_3_2) + 0.46985*std::cos(phi_3_1) - 0.171*std::cos(phi_3_2) + 0.171)*(0.00491831979992485*std::sin(phi_3_1)*std::sin(phi_3_2) - 0.0176126030801545*std::sin(phi_3_1)*std::cos(phi_3_2) - 0.0674285673694518*std::sin(phi_3_1) - 0.0249079821446487*std::sin(phi_3_2)*std::cos(phi_3_1) - 0.00347777728257093*std::cos(phi_3_1)*std::cos(phi_3_2) - 0.0133144168824206*std::cos(phi_3_1) - 0.0095547354815896*std::cos(phi_3_2) - 0.0116809205493283) + k_f*std::pow(pwm_3,2)*(-0.0270277492163122*std::sin(phi_3_1)*std::sin(phi_3_2) + 0.0191115047510637*std::cos(phi_3_1)*std::cos(phi_3_2) + 0.0731670031836881*std::cos(phi_3_1) - 0.00695555456514185*std::cos(phi_3_2) + 0.040539942895125)*(-0.1209152595829*std::sin(phi_3_1)*std::sin(phi_3_2) + 0.433*std::sin(phi_3_1)*std::cos(phi_3_2) + 0.433*std::sin(phi_3_1) + 0.61235447250755*std::sin(phi_3_2)*std::cos(phi_3_1) + 0.0855*std::cos(phi_3_1)*std::cos(phi_3_2) + 0.0855*std::cos(phi_3_1) + 0.2349*std::cos(phi_3_2) - 0.2349);
wrench_alloc.row(4) <<  k_f*std::pow(pwm_1,2)*(-0.5*std::sin(phi_1_1)*std::cos(phi_1_2) - 0.5*std::sin(phi_1_1) - 0.707106781186547*std::sin(phi_1_2)*std::cos(phi_1_1))*(0.0270277492163122*std::sin(phi_1_1)*std::sin(phi_1_2) - 0.0191115047510637*std::cos(phi_1_1)*std::cos(phi_1_2) - 0.0731670031836881*std::cos(phi_1_1) + 0.00695555456514185*std::cos(phi_1_2) - 0.040539942895125) + k_f*std::pow(pwm_1,2)*(-0.020337878845444*std::sin(phi_1_1)*std::cos(phi_1_2) - 0.0778620870316996*std::sin(phi_1_1) - 0.0287621040931278*std::sin(phi_1_2)*std::cos(phi_1_1) - 0.15073)*(-0.664468242280999*std::sin(phi_1_1)*std::sin(phi_1_2) + 0.46985*std::cos(phi_1_1)*std::cos(phi_1_2) + 0.46985*std::cos(phi_1_1) - 0.171*std::cos(phi_1_2) + 0.171) + k_f*std::pow(pwm_2,2)*(-0.664468242280999*std::sin(phi_2_1)*std::sin(phi_2_2) + 0.46985*std::cos(phi_2_1)*std::cos(phi_2_2) + 0.46985*std::cos(phi_2_1) - 0.171*std::cos(phi_2_2) + 0.171)*(-0.00851933523238445*std::sin(phi_2_1)*std::sin(phi_2_2) + 0.010168939422722*std::sin(phi_2_1)*std::cos(phi_2_2) + 0.0389310435158498*std::sin(phi_2_1) + 0.0143810520465639*std::sin(phi_2_2)*std::cos(phi_2_1) + 0.00602407971402052*std::cos(phi_2_1)*std::cos(phi_2_2) + 0.0230627501787894*std::cos(phi_2_1) + 0.0165509658044223*std::cos(phi_2_2) - 0.127136098035242) + k_f*std::pow(pwm_2,2)*(0.0270277492163122*std::sin(phi_2_1)*std::sin(phi_2_2) - 0.0191115047510637*std::cos(phi_2_1)*std::cos(phi_2_2) - 0.0731670031836881*std::cos(phi_2_1) + 0.00695555456514185*std::cos(phi_2_2) - 0.040539942895125)*(-0.209445028587455*std::sin(phi_2_1)*std::sin(phi_2_2) + 0.25*std::sin(phi_2_1)*std::cos(phi_2_2) + 0.25*std::sin(phi_2_1) + 0.353553390593274*std::sin(phi_2_2)*std::cos(phi_2_1) + 0.1481*std::cos(phi_2_1)*std::cos(phi_2_2) + 0.1481*std::cos(phi_2_1) + 0.4069*std::cos(phi_2_2) - 0.4069) + k_f*std::pow(pwm_3,2)*(-0.664468242280999*std::sin(phi_3_1)*std::sin(phi_3_2) + 0.46985*std::cos(phi_3_1)*std::cos(phi_3_2) + 0.46985*std::cos(phi_3_1) - 0.171*std::cos(phi_3_2) + 0.171)*(0.00851933523238445*std::sin(phi_3_1)*std::sin(phi_3_2) + 0.010168939422722*std::sin(phi_3_1)*std::cos(phi_3_2) + 0.0389310435158498*std::sin(phi_3_1) + 0.0143810520465639*std::sin(phi_3_2)*std::cos(phi_3_1) - 0.00602407971402052*std::cos(phi_3_1)*std::cos(phi_3_2) - 0.0230627501787894*std::cos(phi_3_1) - 0.0165509658044223*std::cos(phi_3_2) + 0.268166098035242) + k_f*std::pow(pwm_3,2)*(0.0270277492163122*std::sin(phi_3_1)*std::sin(phi_3_2) - 0.0191115047510637*std::cos(phi_3_1)*std::cos(phi_3_2) - 0.0731670031836881*std::cos(phi_3_1) + 0.00695555456514185*std::cos(phi_3_2) - 0.040539942895125)*(0.209445028587455*std::sin(phi_3_1)*std::sin(phi_3_2) + 0.25*std::sin(phi_3_1)*std::cos(phi_3_2) + 0.25*std::sin(phi_3_1) + 0.353553390593274*std::sin(phi_3_2)*std::cos(phi_3_1) - 0.1481*std::cos(phi_3_1)*std::cos(phi_3_2) - 0.1481*std::cos(phi_3_1) - 0.4069*std::cos(phi_3_2) + 0.4069);
wrench_alloc.row(5) <<  k_f*std::pow(pwm_1,2)*(-0.5*std::sin(phi_1_1)*std::cos(phi_1_2) - 0.5*std::sin(phi_1_1) - 0.707106781186547*std::sin(phi_1_2)*std::cos(phi_1_1))*(0.00983663959984971*std::sin(phi_1_1)*std::sin(phi_1_2) - 0.00695555456514185*std::cos(phi_1_1)*std::cos(phi_1_2) - 0.0266288337648413*std::cos(phi_1_1) - 0.0191115047510637*std::cos(phi_1_2) + 0.229040012685816) + k_f*std::pow(pwm_1,2)*(0.020337878845444*std::sin(phi_1_1)*std::cos(phi_1_2) + 0.0778620870316996*std::sin(phi_1_1) + 0.0287621040931278*std::sin(phi_1_2)*std::cos(phi_1_1) + 0.15073)*(0.241830519165799*std::sin(phi_1_1)*std::sin(phi_1_2) - 0.171*std::cos(phi_1_1)*std::cos(phi_1_2) - 0.171*std::cos(phi_1_1) - 0.46985*std::cos(phi_1_2) + 0.46985) + k_f*std::pow(pwm_2,2)*(-0.209445028587455*std::sin(phi_2_1)*std::sin(phi_2_2) + 0.25*std::sin(phi_2_1)*std::cos(phi_2_2) + 0.25*std::sin(phi_2_1) + 0.353553390593274*std::sin(phi_2_2)*std::cos(phi_2_1) + 0.1481*std::cos(phi_2_1)*std::cos(phi_2_2) + 0.1481*std::cos(phi_2_1) + 0.4069*std::cos(phi_2_2) - 0.4069)*(-0.00491831979992485*std::sin(phi_2_1)*std::sin(phi_2_2) - 0.0176126030801545*std::sin(phi_2_1)*std::cos(phi_2_2) - 0.0674285673694518*std::sin(phi_2_1) - 0.0249079821446487*std::sin(phi_2_2)*std::cos(phi_2_1) + 0.00347777728257093*std::cos(phi_2_1)*std::cos(phi_2_2) + 0.0133144168824206*std::cos(phi_2_1) + 0.0095547354815896*std::cos(phi_2_2) - 0.244639079450672) + k_f*std::pow(pwm_2,2)*(-0.1209152595829*std::sin(phi_2_1)*std::sin(phi_2_2) - 0.433*std::sin(phi_2_1)*std::cos(phi_2_2) - 0.433*std::sin(phi_2_1) - 0.61235447250755*std::sin(phi_2_2)*std::cos(phi_2_1) + 0.0855*std::cos(phi_2_1)*std::cos(phi_2_2) + 0.0855*std::cos(phi_2_1) + 0.2349*std::cos(phi_2_2) - 0.2349)*(0.00851933523238445*std::sin(phi_2_1)*std::sin(phi_2_2) - 0.010168939422722*std::sin(phi_2_1)*std::cos(phi_2_2) - 0.0389310435158498*std::sin(phi_2_1) - 0.0143810520465639*std::sin(phi_2_2)*std::cos(phi_2_1) - 0.00602407971402052*std::cos(phi_2_1)*std::cos(phi_2_2) - 0.0230627501787894*std::cos(phi_2_1) - 0.0165509658044223*std::cos(phi_2_2) + 0.127136098035242) + k_f*std::pow(pwm_3,2)*(-0.1209152595829*std::sin(phi_3_1)*std::sin(phi_3_2) + 0.433*std::sin(phi_3_1)*std::cos(phi_3_2) + 0.433*std::sin(phi_3_1) + 0.61235447250755*std::sin(phi_3_2)*std::cos(phi_3_1) + 0.0855*std::cos(phi_3_1)*std::cos(phi_3_2) + 0.0855*std::cos(phi_3_1) + 0.2349*std::cos(phi_3_2) - 0.2349)*(-0.00851933523238445*std::sin(phi_3_1)*std::sin(phi_3_2) - 0.010168939422722*std::sin(phi_3_1)*std::cos(phi_3_2) - 0.0389310435158498*std::sin(phi_3_1) - 0.0143810520465639*std::sin(phi_3_2)*std::cos(phi_3_1) + 0.00602407971402052*std::cos(phi_3_1)*std::cos(phi_3_2) + 0.0230627501787894*std::cos(phi_3_1) + 0.0165509658044223*std::cos(phi_3_2) - 0.268166098035242) + k_f*std::pow(pwm_3,2)*(-0.00491831979992485*std::sin(phi_3_1)*std::sin(phi_3_2) + 0.0176126030801545*std::sin(phi_3_1)*std::cos(phi_3_2) + 0.0674285673694518*std::sin(phi_3_1) + 0.0249079821446487*std::sin(phi_3_2)*std::cos(phi_3_1) + 0.00347777728257093*std::cos(phi_3_1)*std::cos(phi_3_2) + 0.0133144168824206*std::cos(phi_3_1) + 0.0095547354815896*std::cos(phi_3_2) + 0.0116809205493283)*(0.209445028587455*std::sin(phi_3_1)*std::sin(phi_3_2) + 0.25*std::sin(phi_3_1)*std::cos(phi_3_2) + 0.25*std::sin(phi_3_1) + 0.353553390593274*std::sin(phi_3_2)*std::cos(phi_3_1) - 0.1481*std::cos(phi_3_1)*std::cos(phi_3_2) - 0.1481*std::cos(phi_3_1) - 0.4069*std::cos(phi_3_2) + 0.4069);