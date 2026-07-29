#pragma once

#include <stdbool.h>
#include <stdint.h>

#define HG_PARTICLE_CAPACITY 300
#define HG_DEFAULT_PARTICLE_COUNT 300
#define HG_DEFAULT_MAX_ACTIVE 80
#define HG_DEFAULT_CONSTRAINT_ITERATIONS 3
#define HG_PHYSICS_HZ 20
#define HG_PHYSICS_DT (1.0f / (float)HG_PHYSICS_HZ)

typedef enum {
    HG_PARTICLE_UNUSED = 0,
    HG_PARTICLE_UPPER_SLEEPING,
    HG_PARTICLE_UPPER_ACTIVE,
    HG_PARTICLE_FALLING,
    HG_PARTICLE_LOWER_ACTIVE,
    HG_PARTICLE_LOWER_SLEEPING,
} hg_particle_state_t;

typedef struct {
    float x;
    float y;
    float vx;
    float vy;
    uint8_t size;
    uint8_t state;
    uint8_t stable_ticks;
    uint8_t color_phase;
    int16_t grid_next;
} hg_particle_t;

typedef struct {
    uint16_t total;
    uint16_t upper;
    uint16_t falling;
    uint16_t lower;
    uint16_t active;
    uint16_t sleeping;
    uint16_t transferred;
    uint16_t throat_particles;
    uint16_t throat_active;
    uint16_t flow_stall_ms;
    uint16_t stall_detect_count;
    uint16_t unblock_count;
    uint16_t upper_continuity_fault_count;
    uint16_t detected_void_count;
    uint16_t bridge_break_count;
    uint16_t upper_surface_active;
    uint16_t upper_subsurface_active;
    float effective_throat_width;
    bool settled;
} hg_physics_stats_t;

void hg_physics_init(void);
void hg_physics_reset(void);
void hg_physics_step(float gravity_x, float gravity_y, float progress,
                     bool allow_flow);
void hg_physics_set_limits(uint8_t max_active, uint8_t constraint_iterations);
void hg_physics_request_particle_count(uint16_t particle_count);
void hg_physics_wake_surface(void);

const hg_particle_t *hg_physics_particles(void);
uint16_t hg_physics_particle_count(void);
hg_physics_stats_t hg_physics_stats(void);

float hg_geometry_left_boundary(int y);
float hg_geometry_right_boundary(int y);
