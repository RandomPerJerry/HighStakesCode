#ifndef _AUTO_H_
#define _AUTO_H_

// Autonomous routine declarations
enum class AutonomousMode {
    SKILLS,
    RED_RING,
    RED_STAKE,
    BLUE_RING,
    BLUE_STAKE,      
    TEST,
    SCREW
};

extern AutonomousMode current_auto;

void skills_auto();
void red_ring_auto();
void red_stake_auto();
void blue_ring_auto();
void blue_stake_auto();
void test_auto();


#endif // _AUTO_H_ 