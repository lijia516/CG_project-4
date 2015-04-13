#pragma warning(disable : 4786)

#include "particleSystem.h"
#include "modelerdraw.h"

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <vector>
#include <map>

using namespace std;

static float prevT;

Vec3f ParticleSystem::gravity = Vec3f(0, -9.8f, 0);
int ParticleSystem::particleNum = 5;
int ParticleSystem::particleReal = 0;

/***************
 * Constructors
 ***************/

ParticleSystem::ParticleSystem() 
{
	// TODO
    
    time = 0;
    
}




/*************
 * Destructor
 *************/

ParticleSystem::~ParticleSystem() 
{
	// TODO
    
}


/******************
 * Simulation fxns
 ******************/

/** Start the simulation */
void ParticleSystem::startSimulation(float t)
{
	// TODO
    
    
	// These values are used by the UI ...
	// negative bake_end_time indicates that simulation
	// is still progressing, and allows the
	// indicator window above the time slider
	// to correctly show the "baked" region
	// in grey.
	bake_end_time = -1;
	simulate = true;
	dirty = true;

}

/** Stop the simulation */
void ParticleSystem::stopSimulation(float t)
{
	// TODO

    Particle* p = new Particle();
    p->position = Vec3f(particleOrigin);
    p->force = Vec3f(gravity);
    particles.push_back(p);
    time  = t;
    
	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Reset the simulation */
void ParticleSystem::resetSimulation(float t)
{
	// TODO

	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Compute forces and update particles **/
void ParticleSystem::computeForcesAndUpdateParticles(float t)
{
	// TODO
    
   //  std::cout <<"time: " << t << std::endl;
    
    if (isSimulate()) {
    
        if (t - time > 0.5) {
        
            time = t;
            
            if  (particleReal < 5) {
            
             //   std::cout <<"3,3,0: " << std::endl;
                
                Particle* p = new Particle();
                p->position = Vec3f(particleOrigin);
                p->force = Vec3f(gravity);
                particles.push_back(p);
                particleReal++;
            }
        
        }
        
        float deltaT = t - prevT;

        typedef vector<Particle*>::const_iterator iter;
        

        for(iter i = particles.begin(); i != particles.end(); ++i) {
            
        //    std::cout << (*i)->position[0] << "," << (*i)->position[1] << "," << (*i)->position[2] << std::endl;
            
            (*i)->velocity[0] += (*i)->force[0] * 1.0 / (*i)->mass * deltaT * 0.1;
            (*i)->velocity[1] += (*i)->force[1] * 1.0 / (*i)->mass * deltaT * 0.1;
            (*i)->velocity[2] += (*i)->force[2] * 1.0 / (*i)->mass * deltaT * 0.1;
            
            (*i)->position[0] += (*i)->velocity[0] * deltaT;
            (*i)->position[1] += (*i)->velocity[1] * deltaT;
            (*i)->position[2] += (*i)->velocity[2] * deltaT;
            
            std::cout << (*i)->position[0] << "," << (*i)->position[1] << "," << (*i)->position[2] << std::endl;

        }
        bakeParticles(t);
    }
    
	// Debugging info
	if( t - prevT > .04 )
		printf("(!!) Dropped Frame %lf (!!)\n", t-prevT);
	prevT = t;
}


/** Render particles */
void ParticleSystem::drawParticles(float t)
{
	// TODO
    
    
    int count  = 0;
    
    // state update
    
     if (isSimulate()) {
    
         typedef vector<Particle*>::const_iterator iter;
         
         for(iter i = particles.begin(); i != particles.end(); ++i) {
             
             glPushMatrix();
                glTranslated((*i)->position[0], (*i)->position[1], (*i)->position[2]);
                drawSphere(0.1);
             glPopMatrix();
             
             if ((*i)->position[1] < 0) count ++;
         }
         
         
     } else {
         
         
         std::cout <<"find bake time: " << t << std::endl;
         
         std::vector<Vec3f> current_particles_position;
         std::map<float, std::vector<Vec3f> >::iterator result = bake_particles.find(t);
         if (result != bake_particles.end())
             current_particles_position = (*result).second;
         
         
         
         typedef vector<Vec3f>::const_iterator iter;
         
         for(iter i = current_particles_position.begin(); i != current_particles_position.end(); ++i) {
             
             glPushMatrix();
                glTranslated((*i)[0], (*i)[1], (*i)[2]);
                drawSphere(0.1);
             glPopMatrix();
             
             if ((*i)[1] < 0) count ++;
         }
         
     }
    
   
    particleReal -= count;
    
   // std::cout <<"draw() real number: " <<particleReal << std::endl;
}




/** Adds the current configuration of particles to
  * your data structure for storing baked particles **/
void ParticleSystem::bakeParticles(float t) 
{
	// TODO
    
    std::vector<Vec3f> current_particles_position;
    typedef vector<Particle*>::const_iterator iter;
    
    // state update
    
    for(iter i = particles.begin(); i != particles.end(); ++i) {
        
        Vec3f position = Vec3f((*i)->position[0], (*i)->position[1], (*i)->position[2]);
        
        current_particles_position.push_back(position);
        
    }
    
    bake_particles.insert(std::map<float, std::vector<Vec3f> >::value_type(t, current_particles_position));
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked()
{
	// TODO
    
    bake_particles.clear();
    
}




