#include "Test.h"
#include "Pattern.h"
class InterpolationTestsQ7:public Client::Suite
    {
        public:
            InterpolationTestsQ7(Testing::testID_t id);
            virtual void setUp(Testing::testID_t,std::vector<Testing::param_t>& params,Client::PatternMgr *mgr);
            virtual void tearDown(Testing::testID_t,Client::PatternMgr *mgr);
        private:
            #include "InterpolationTestsQ7_decl.h"
            
            Client::Pattern<q31_t> input;
            Client::Pattern<q7_t> y;
            Client::Pattern<int16_t> config;
            Client::LocalPattern<q7_t> output;
            // Reference patterns are not loaded when we are in dump mode
            Client::RefPattern<q7_t> ref;

            arm_bilinear_interp_instance_q7 SBI;

    };
