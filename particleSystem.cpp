#pragma warning(disable : 4786)

#include "particleSystem.h"
#include "modelerdraw.h"

#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <vector>
#include <map>

#include <FL/gl.h>

using namespace std;

static float prevT;
float ParticleSystem::particleRadius = 0.15;
Vec3f ParticleSystem::gravity = Vec3f(0, -9.8f, 0);
float ParticleSystem::airResistance = 1.5;
int ParticleSystem::particleNum = 500;
int ParticleSystem::particleReal = 0;
float ParticleSystem::spring_K = 500;
int ParticleSystem::particleNum_ponyTail = 8;

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
    particles.clear();
    
}


/******************
 * Simulation fxns
 ******************/

/** Start the simulation */
void ParticleSystem::startSimulation(float t)
{
	// TODO
    
    // pony tail
    ponyTail_particles.resize(particleNum_ponyTail, NULL);
    Particle* p = new Particle();
    p->position = Vec3f(0,3,3);
    p->velocity = Vec3f(0,0,0);
    p->force = Vec3f(0,0,0);
    ponyTail_particles[0] = p;
    
    
    for (int i = 1; i < particleNum_ponyTail; i++) {
        
        Particle* p = new Particle();
        p->position = Vec3f(0 + i * 0.5, 3 + i * 0.1, 3 + i * 0.1);
        p->velocity = Vec3f(0.35,0.3,0);
        p->force = Vec3f(gravity);
        ponyTail_particles[i] = p;
    }
    
    int i = 1;
    std::cout << "position[i]: "<<ponyTail_particles[i]->position[0] << "," << ponyTail_particles[i]->position[1] << "," << ponyTail_particles[i]->position[2] << std::endl;
    
    std::cout << "position[i - 1]: "<<ponyTail_particles[i- 1]->position[0] << "," << ponyTail_particles[i- 1]->position[1] << "," << ponyTail_particles[i- 1]->position[2] << std::endl;
    
    
    
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
    particles.clear();
    
	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Reset the simulation */
void ParticleSystem::resetSimulation(float t)
{
	// TODO

    particles.clear();
    
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
        
        
        if (t - time > 0.3) {
        
            time = t;
            
            if  (particleReal < particleNum) {
                
                Particle* p = new Particle();
                p->position = Vec3f(particleOrigin);
                p->velocity = Vec3f(0, -2, 0);
                p->force = Vec3f(gravity);
                particles.push_back(p);
                particleReal++;
            }
        }
        
        float deltaT = t - prevT;

        typedef vector<Particle*>::const_iterator iter;
        
        for(iter i = particles.begin(); i != particles.end(); ++i) {
            
            // F = 1/2 * C * S * V^2
            Vec3f airResistance_cur = Vec3f(0,0,0);
           
            airResistance_cur[0] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[0] * (*i)->velocity[0];
            airResistance_cur[1] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[1] * (*i)->velocity[1];
            airResistance_cur[2] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[2] * (*i)->velocity[2];
            
            if ((*i)->velocity[0] > 0) {
                
                (*i)->force[0] -= airResistance_cur[0];
            } else {
                
                 (*i)->force[0] += airResistance_cur[0];
            }
            
            if ((*i)->velocity[1] > 0) {
                
                (*i)->force[1] -= airResistance_cur[1];
            } else {
                
                (*i)->force[1] += airResistance_cur[1];
            }
            
            if ((*i)->velocity[2] > 0) {
                
                (*i)->force[2] -= airResistance_cur[2];
            } else {
                
                (*i)->force[2] += airResistance_cur[2];
            }
            
            
         //   (*i)->force[1] += airResistance_cur[1];
         //   (*i)->force[2] += airResistance_cur[2];
            
            
            (*i)->velocity[0] += (*i)->force[0] * 1.0 / (*i)->mass * deltaT * 0.1;
            (*i)->velocity[1] += (*i)->force[1] * 1.0 / (*i)->mass * deltaT * 0.1;
            (*i)->velocity[2] += (*i)->force[2] * 1.0 / (*i)->mass * deltaT * 0.1;
            
            (*i)->position[0] += (*i)->velocity[0] * deltaT;
            (*i)->position[1] += (*i)->velocity[1] * deltaT;
            (*i)->position[2] += (*i)->velocity[2] * deltaT;

        }

        bakeParticles(t);
        
        
        ponyTail_computeForcesAndUpdateParticles(t);
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
    
    
    // state update
    
     if (isSimulate()) {
         
         typedef vector<Particle*>::const_iterator iter;
         
         for (iter i = particles.begin(); i != particles.end(); i++) {
 
             if ((*i)->position[1] < 0) {
                 
                 if ((*i) -> position[1] < 0) {
                     
                     particles.erase(i);
                     particleReal--;
                 }
             }
         }
         
         for(iter i = particles.begin(); i != particles.end(); ++i) {
             
             glPushMatrix();
                glTranslated((*i)->position[0], (*i)->position[1], (*i)->position[2]);
                drawSphere(particleRadius);
             glPopMatrix();
         }
       
         
         
         for(iter i = ponyTail_particles.begin(); i != ponyTail_particles.end(); ++i) {
             
             
             glPushMatrix();
                glTranslated((*i)->position[0], (*i)->position[1], (*i)->position[2]);
                drawSphere(particleRadius);
             glPopMatrix();
             
             
             if (i != ponyTail_particles.begin()) {
             
                 glLineWidth(2.5);
                 glColor3f(0.0, 0.0, 1.0);
                 glBegin(GL_LINES);
                    glVertex3f((*i)->position[0], (*i)->position[1], (*i)->position[2]);
                    glVertex3f((*(i-1))->position[0], (*(i-1))->position[1], (*(i-1))->position[2]);
                 glEnd();
             }
             
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
             
         }
         
         
         std::vector<Vec3f> current_particlesPony_position;
         std::map<float, std::vector<Vec3f> >::iterator result2 = bake_particlesPony.find(t);
         if (result2 != bake_particlesPony.end())
             current_particlesPony_position = (*result2).second;
         
         typedef vector<Vec3f>::const_iterator iter;
         
         for(iter i = current_particlesPony_position.begin(); i != current_particlesPony_position.end(); ++i) {
             
             glPushMatrix();
                glTranslated((*i)[0], (*i)[1], (*i)[2]);
                drawSphere(particleRadius);
             glPopMatrix();
             
             
             if (i != current_particlesPony_position.begin()) {
                 
                 glLineWidth(2.5);
                 glColor3f(0.0, 0.0, 1.0);
                 glBegin(GL_LINES);
                 glVertex3f((*i)[0], (*i)[1], (*i)[2]);
                 glVertex3f((*(i-1))[0], (*(i-1))[1], (*(i-1))[2]);
                 glEnd();
             }
             
         }
         
         
     }
    
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
    
    
    std::vector<Vec3f> current_particlesPony_position;
    
    for(iter i = ponyTail_particles.begin(); i != ponyTail_particles.end(); ++i) {
        
        Vec3f position = Vec3f((*i)->position[0], (*i)->position[1], (*i)->position[2]);
        
        current_particlesPony_position.push_back(position);
        
    }
    
    bake_particlesPony.insert(std::map<float, std::vector<Vec3f> >::value_type(t, current_particlesPony_position));
    
    
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked()
{
	// TODO
    
    bake_particles.clear();
    
}


void ParticleSystem::ponyTail_computeForcesAndUpdateParticles(float t)
{
    // TODO
    
    if (isSimulate()) {
        
        
        float deltaT = t - prevT;
        
        typedef vector<Particle*>::const_iterator iter;
        
        int index = 0;
        
        for(iter i = ponyTail_particles.begin() + 1; i != ponyTail_particles.end(); ++i) {
            
            index++;
            
            // F = 1/2 * C * S * V^2
            Vec3f airResistance_cur = Vec3f(0,0,0);
            airResistance_cur[0] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[0] * (*i)->velocity[0];
            airResistance_cur[1] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[1] * (*i)->velocity[1];
            airResistance_cur[2] = 0.5 * airResistance * 4 * 3.1415926 * particleRadius * particleRadius * 0.5 * (*i)->velocity[2] * (*i)->velocity[2];
            
            (*i)->force[0] += airResistance_cur[0];
            (*i)->force[1] += airResistance_cur[1];
            (*i)->force[2] += airResistance_cur[2];
            
            if (index < particleNum_ponyTail - 1) {
                
                
                Vec3f left = Vec3f((*i)->position[0] - (*(i - 1))->position[0], (*i)->position[1] - (*(i - 1))->position[1], (*i)->position[2] - (*(i - 1))->position[2]);
                
                float distance_left = left.length();
                left.normalize();
                Vec3f force_left = spring_K * (1 - distance_left) * left;
                Vec3f right = Vec3f((*i)->position[0] - (*(i+1))->position[0], (*i)->position[1] - (*(i+1))->position[1], (*i)->position[2] - (*(i+1))->position[2]);
                float distance_right = right.length();
                right.normalize();
                Vec3f force_right = spring_K * (1 - distance_right) * right;
                
                (*i)->force[0] += force_left[0];
                (*i)->force[0] += force_right[0];
                
                (*i)->force[1] += force_left[1];
                (*i)->force[1] += force_right[1];
                
                (*i)->force[2] += force_left[2];
                (*i)->force[2] += force_right[2];
                
            } else {
                
                
                Vec3f left = Vec3f((*i)->position[0] - (*(i - 1))->position[0], (*i)->position[1] - (*(i - 1))->position[1], (*i)->position[2] - (*(i - 1))->position[2]);
                float distance_left = left.length();
                left.normalize();
                Vec3f force_left = spring_K * (1 - distance_left) * left;
                
                (*i)->force[0]  +=  force_left[0];
                (*i)->force[1]  +=  force_left[1];
                (*i)->force[2]  +=  force_left[2];
            }

            
            if ((*i)->force.length() > 10) {
                (*i)->force.normalize();
                (*i)->force *= 10;
            }
            
            (*i)->velocity[0] += (*i)->force[0] * 1.0 / (*i)->mass * deltaT * 0.05;
            (*i)->velocity[1] += (*i)->force[1] * 1.0 / (*i)->mass * deltaT * 0.05;
            (*i)->velocity[2] += (*i)->force[2] * 1.0 / (*i)->mass * deltaT * 0.05;
            
            (*i)->position[0] += (*i)->velocity[0] * deltaT;
            (*i)->position[1] += (*i)->velocity[1] * deltaT;
            (*i)->position[2] += (*i)->velocity[2] * deltaT;

        }
  
        
        bakeParticles(t);
    }
    
    // Debugging info
    if( t - prevT > .04 )
        printf("(!!) Dropped Frame %lf (!!)\n", t-prevT);
    prevT = t;
}
