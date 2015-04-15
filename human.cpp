// The sample robotarm model.  You should build a file
// very similar to this for when you make your model.
#pragma warning (disable : 4305)
#pragma warning (disable : 4244)
#pragma warning(disable : 4786)
#pragma warning (disable : 4312)

#include "humanInfor.h"
#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include "particleSystem.h"

#include "mat.h"
#include <FL/gl.h>
#include <cstdlib>

using namespace std;

#define M_DEFAULT 2.0f
#define M_OFFSET 3.0f
#define P_OFFSET 0.3f
#define MAX_VEL 200
#define MIN_STEP 0.1

Vec4f ParticleSystem::particleOrigin = Vec4f(0,0,0,1);


// This is a list of the controls for the RobotArm
// We'll use these constants to access the values 
// of the controls from the user interface.
enum RobotArmControls
{
    
    PELVIS_R=0, LTHIGH_RX, LTHIGH_RY,LTHIGH_RZ, RTHIGH_RX, RTHIGH_RY, RTHIGH_RZ, ABDOMEN, LSHIN_RX, RSHIN_RX, LFOOT_RX, LFOOT_RY, LFOOT_RZ, RFOOT_RX, RFOOT_RY, RFOOT_RZ,
    
    PARTICLE_COUNT, NUMCONTROLS,
    LTHIGH_H, RTHIGH_H, LSHIN_H, RSHIN_H,
    
    //BASE_ROTATION=0, LOWER_TILT, UPPER_TILT, CLAW_ROTATION,
      //  BASE_LENGTH, LOWER_LENGTH, UPPER_LENGTH, PARTICLE_COUNT, NUMCONTROLS,
};

void ground(float h);
void thigh(float h);
void pelvis(float h);
void shin(float h);
void foot(float h);

void y_box(float h);
Mat4f glGetMatrix(GLenum pname);
Vec3f getWorldPoint(Mat4f matCamXforms);

// To make a RobotArm, we inherit off of ModelerView
class Human : public ModelerView
{
public:
    Human(int x, int y, int w, int h, char *label)
        : ModelerView(x,y,w,h,label) {}
    virtual void draw();
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createHuman(int x, int y, int w, int h, char *label)
{ 
    return new Human(x,y,w,h,label);
}

// We'll be getting the instance of the application a lot; 
// might as well have it as a macro.
#define VAL(x) (ModelerApplication::Instance()->GetControlValue(x))


// Utility function.  Use glGetMatrix(GL_MODELVIEW_MATRIX) to retrieve
//  the current ModelView matrix.
Mat4f glGetMatrix(GLenum pname)
{
    GLfloat m[16];
    glGetFloatv(pname, m);
    Mat4f matCam(m[0],  m[1],  m[2],  m[3],
                            m[4],  m[5],  m[6],  m[7],
                            m[8],  m[9],  m[10], m[11],
                            m[12], m[13], m[14], m[15] );

    // because the matrix GL returns is column major...
    return matCam.transpose();
}





// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out RobotArm
void Human::draw()
{
	/* pick up the slider values */

    
    float pelvis_r = VAL( PELVIS_R );
    float lthigh_rx = VAL( LTHIGH_RX );
    float lthigh_ry = VAL( LTHIGH_RY );
    float lthigh_rz = VAL( LTHIGH_RZ );
    float rthigh_rx = VAL( RTHIGH_RX );
    float rthigh_ry = VAL( RTHIGH_RY );
    float rthigh_rz = VAL( RTHIGH_RZ );
    
    float lshin_rx = VAL( LSHIN_RX );
    float rshin_rx = VAL( RSHIN_RX );
    
    
    float lfoot_rx = VAL( LFOOT_RX );
    float lfoot_ry = VAL( LFOOT_RY );
    float lfoot_rz = VAL( LFOOT_RZ );
    
    
    float rfoot_rx = VAL( RFOOT_RX );
    float rfoot_ry = VAL( RFOOT_RY );
    float rfoot_rz = VAL( RFOOT_RZ );
    

    // This call takes care of a lot of the nasty projection 
    // matrix stuff
    ModelerView::draw();

    // Save the camera transform that was applied by
    // ModelerView::draw() above.
    // While we're at it, save an inverted copy of this matrix.  We'll
    // need it later.
    Mat4f matCam = glGetMatrix( GL_MODELVIEW_MATRIX );
    Mat4f matCamInverse = matCam.inverse();



	static GLfloat lmodel_ambient[] = {0.4,0.4,0.4,1.0};

	// define the model

	ground(-0.2);

    glTranslatef( 0.0, 2.5, 0.0 );
    
    glPushMatrix();
    
        glRotatef(pelvis_r, 0.0, 1.0, 0.0 );
        pelvis(1);
            glPushMatrix();
                glTranslatef( 0.5, 0.0, 0.0 );
                glRotatef(lthigh_rx, 0.0, 0.0, 1.0 );
                glRotatef(lthigh_ry, 0.0, 1.0, 0.0 );
                glRotatef(lthigh_rz, 1.0, 0.0, 0.0 );
                thigh(1);
                glPushMatrix();
                    glTranslatef( 0.0, -1, 0.0 );
                    glRotatef(lshin_rx, 1.0, 0.0, 0.0 );
                    shin(1);
                    glPushMatrix();
                        glTranslatef( 0.0, -1.1, 0.0 );
                        glRotatef(lfoot_rx, 0.0, 0.0, 1.0 );
                        glRotatef(lfoot_ry, 0.0, 1.0, 0.0 );
                        glRotatef(lfoot_rz, 1.0, 0.0, 0.0 );
                        foot(2);
                    glPopMatrix();
    
                glPopMatrix();
            glPopMatrix();
    
            glPushMatrix();
                glTranslatef( -0.5, 0.0, 0.0 );
                glRotatef(rthigh_rx, 0.0, 0.0, 1.0 );
                glRotatef(rthigh_ry, 0.0, 1.0, 0.0 );
                glRotatef(rthigh_rz, 1.0, 0.0, 0.0 );
                thigh(1);
                glPushMatrix();
                    glTranslatef( 0.0, -1, 0.0 );
                    glRotatef(rshin_rx, 1.0, 0.0, 0.0 );
                    shin(1);
                    glPushMatrix();
                        glTranslatef( 0.0, -1.1, 0.0 );
                        glRotatef(rfoot_rx, 0.0, 0.0, 1.0 );
                        glRotatef(rfoot_ry, 0.0, 1.0, 0.0 );
                        glRotatef(rfoot_rz, 1.0, 0.0, 0.0 );
                        foot(2);
                    glPopMatrix();
                glPopMatrix();
            glPopMatrix();
    glPopMatrix();
    
    
    /*
    glTranslatef( 0.0, 0.8, 0.0 );			// move to the top of the base
    glRotatef( theta, 0.0, 1.0, 0.0 );		// turn the whole assembly around the y-axis. 
	rotation_base(h1);						// draw the rotation base

    glTranslatef( 0.0, h1, 0.0 );			// move to the top of the base
	glPushMatrix();
			glTranslatef( 0.5, h1, 0.6 );	
	glPopMatrix();
    glRotatef( phi, 0.0, 0.0, 1.0 );		// rotate around the z-axis for the lower arm
	glTranslatef( -0.1, 0.0, 0.4 );
	lower_arm(h2);							// draw the lower arm

    glTranslatef( 0.0, h2, 0.0 );			// move to the top of the lower arm
    glRotatef( psi, 0.0, 0.0, 1.0 );		// rotate  around z-axis for the upper arm
	upper_arm(h3);							// draw the upper arm

	glTranslatef( 0.0, h3, 0.0 );
	glRotatef( cr, 0.0, 0.0, 1.0 );
	claw(1.0);
     
     */

    Mat4f particleXform = matCamInverse * glGetMatrix(GL_MODELVIEW_MATRIX);
    ParticleSystem::particleOrigin = particleXform * Vec4f(0,0,0,1);
    
  //  std::cout<<"ori:" << ParticleSystem::particleOrigin[0] << "," << ParticleSystem::particleOrigin[1] << "," << ParticleSystem::particleOrigin[2] << std::endl;

	//*** DON'T FORGET TO PUT THIS IN YOUR OWN CODE **/
	endDraw();
}

void ground(float h) 
{
	glDisable(GL_LIGHTING);
	glColor3f(0.65,0.45,0.2);
	glPushMatrix();
	glScalef(30,0,30);
	y_box(h);
	glPopMatrix();
	glEnable(GL_LIGHTING);
}



void pelvis(float pelvis_r) {
    
    setDiffuseColor( 0.25, 0.25, 0.25 );
    setAmbientColor( 0.25, 0.25, 0.25 );
    
    glColor3f(0,0,1);
    
    glPushMatrix();
        glRotatef(90,0,1,0);
        glScalef(trunk_fat*1.33 * 3,pelvis_len/2 * 3,trunk_width/2*1.5 * 3);
        drawSphere(1);
    glPopMatrix();
    
}


void thigh(float h) {
    
    setDiffuseColor( 0.25, 0.25, 0.25 );
    setAmbientColor( 0.25, 0.25, 0.25 );
    
     glColor3f(0,1,0);
    
     glPushMatrix();
        glScalef(leg_fat*1.75 * 3, thigh_len/2 * 3, leg_fat*1.75 * 3);
        glTranslatef(0, -1, 0);
        drawSphere(1);
    glPopMatrix();
    
}

void shin(float h) {
    
    glColor3f(1,0,0);
    
    setDiffuseColor( 0.25, 0.25, 0.25 );
    setAmbientColor( 0.25, 0.25, 0.25 );
    
    glPushMatrix();
        glScalef(leg_fat * 3, shin_len/2 * 3, leg_fat * 3);
        glTranslatef(0, -1, 0);
        drawSphere(1);
    glPopMatrix();
    
}

void foot(float h) {
    
    setDiffuseColor( 0.25, 0.25, 0.25 );
    setAmbientColor( 0.25, 0.25, 0.25 );
    
    glPushMatrix();
        glRotatef(90,0,1,0);
        glTranslatef(-heel_len - 0.05, -ankle_height, -0.125);
        glScalef(foot_len/2 * 3,.02 * 3, foot_len/4 * 3);
        drawBox(2,2,2);
    glPopMatrix();
}







void y_box(float h) {

	glBegin( GL_QUADS );

	glNormal3d( 1.0 ,0.0, 0.0);			// +x side
	glVertex3d( 0.25,0.0, 0.25);
	glVertex3d( 0.25,0.0,-0.25);
	glVertex3d( 0.25,  h,-0.25);
	glVertex3d( 0.25,  h, 0.25);

	glNormal3d( 0.0 ,0.0, -1.0);		// -z side
	glVertex3d( 0.25,0.0,-0.25);
	glVertex3d(-0.25,0.0,-0.25);
	glVertex3d(-0.25,  h,-0.25);
	glVertex3d( 0.25,  h,-0.25);

	glNormal3d(-1.0, 0.0, 0.0);			// -x side
	glVertex3d(-0.25,0.0,-0.25);
	glVertex3d(-0.25,0.0, 0.25);
	glVertex3d(-0.25,  h, 0.25);
	glVertex3d(-0.25,  h,-0.25);

	glNormal3d( 0.0, 0.0, 1.0);			// +z side
	glVertex3d(-0.25,0.0, 0.25);
	glVertex3d( 0.25,0.0, 0.25);
	glVertex3d( 0.25,  h, 0.25);
	glVertex3d(-0.25,  h, 0.25);

	glNormal3d( 0.0, 1.0, 0.0);			// top (+y)
	glVertex3d( 0.25,  h, 0.25);
	glVertex3d( 0.25,  h,-0.25);
	glVertex3d(-0.25,  h,-0.25);
	glVertex3d(-0.25,  h, 0.25);

	glNormal3d( 0.0,-1.0, 0.0);			// bottom (-y)
	glVertex3d( 0.25,0.0, 0.25);
	glVertex3d(-0.25,0.0, 0.25);
	glVertex3d(-0.25,0.0,-0.25);
	glVertex3d( 0.25,0.0,-0.25);

	glEnd();
}

int main()
{
    ModelerControl controls[NUMCONTROLS ];

    
	controls[PELVIS_R ] = ModelerControl("plevis rotation", -180.0, 180.0, 0.1, 0.0 );
    controls[LTHIGH_RX ] = ModelerControl("lthigh x rotation", -180.0, 180.0, 0.1, 0.0 );
    controls[LTHIGH_RY ] = ModelerControl("lthigh y rotation", -45.0, 45.0, 0.1, 0.0 );
    controls[LTHIGH_RZ ] = ModelerControl("lthigh z rotation", -180.0, 180.0, 0.1, 0.0 );
    
    controls[RTHIGH_RX ] = ModelerControl("rthigh x rotation", -180.0, 180.0, 0.1, 0.0 );
    controls[RTHIGH_RY ] = ModelerControl("rthigh y rotation", -45.0, 45.0, 0.1, 0.0 );
    controls[RTHIGH_RZ ] = ModelerControl("rthigh z rotation", -180.0, 180.0, 0.1, 0.0 );
    
    
    controls[LSHIN_RX ] = ModelerControl("lshin z rotation", 0.0, -180.0, 0.1, 0.0 );
    controls[RSHIN_RX ] = ModelerControl("rshin z rotation", 0.0, -180.0, 0.1, 0.0 );
    
    controls[LFOOT_RX ] = ModelerControl("lfoot x rotation", -10.0, 10.0, 0.1, 0.0 );
    controls[LFOOT_RY ] = ModelerControl("lfoot y rotation", -45.0, 45.0, 0.1, 0.0 );
    controls[LFOOT_RZ ] = ModelerControl("lfoot z rotation", -45.0, 45.0, 0.1, 0.0 );
    
    
    
    controls[RFOOT_RX ] = ModelerControl("rfoot x rotation", -10.0, 10.0, 0.1, 0.0 );
    controls[RFOOT_RY ] = ModelerControl("rfoot y rotation", -45.0, 45.0, 0.1, 0.0 );
    controls[RFOOT_RZ ] = ModelerControl("rfoot z rotation", -45.0, 45.0, 0.1, 0.0 );
    
    
 //   controls[LOWER_TILT] = ModelerControl("lower arm tilt (phi)", 15.0, 95.0, 0.1, 55.0 );
//    controls[UPPER_TILT] = ModelerControl("upper arm tilt (psi)", 0.0, 135.0, 0.1, 30.0 );
//	controls[CLAW_ROTATION] = ModelerControl("claw rotation (cr)", -30.0, 180.0, 0.1, 0.0 );
//    controls[BASE_LENGTH] = ModelerControl("base height (h1)", 0.5, 10.0, 0.1, 0.8 );
//    controls[LOWER_LENGTH] = ModelerControl("lower arm length (h2)", 1, 10.0, 0.1, 3.0 );
//    controls[UPPER_LENGTH] = ModelerControl("upper arm length (h3)", 1, 10.0, 0.1, 2.5 );
//    controls[PARTICLE_COUNT] = ModelerControl("particle count (pc)", 0.0, 5.0, 0.1, 5.0 );
    

	// You should create a ParticleSystem object ps here and then
	// call ModelerApplication::Instance()->SetParticleSystem(ps)
	// to hook it up to the animator interface.

    ParticleSystem *ps = new ParticleSystem();
    ModelerApplication::Instance()->SetParticleSystem(ps);
    
    ModelerApplication::Instance()->Init(&createHuman, controls, NUMCONTROLS);
    return ModelerApplication::Instance()->Run();
}
