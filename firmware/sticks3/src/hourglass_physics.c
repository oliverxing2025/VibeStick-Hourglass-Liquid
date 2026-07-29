#include "hourglass_physics.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define HG_CANVAS_W 115
#define HG_CANVAS_H 118
#define HG_CENTER_X 57.0f
#define HG_TOP_Y 10
#define HG_NECK_TOP_Y 55
#define HG_NECK_BOTTOM_Y 62
#define HG_BOTTOM_Y 108
#define HG_THROAT_ZONE_TOP 35
#define HG_THROAT_ZONE_BOTTOM 78
#define HG_THROAT_CORE_TOP 50
#define HG_THROAT_CORE_BOTTOM 67
#define HG_THROAT_BASE_WIDTH 7.0f
#define HG_THROAT_MAX_EXPANSION 0.05f

#define HG_GRID_CELL_SIZE 4
#define HG_GRID_COLS ((HG_CANVAS_W + HG_GRID_CELL_SIZE - 1) / HG_GRID_CELL_SIZE)
#define HG_GRID_ROWS ((HG_CANVAS_H + HG_GRID_CELL_SIZE - 1) / HG_GRID_CELL_SIZE)
#define HG_GRID_CELLS (HG_GRID_COLS * HG_GRID_ROWS)

#define HG_GRAVITY_ACCEL 54.0f
#define HG_MAX_SPEED 54.0f
#define HG_AIR_DAMPING 0.995f
#define HG_CONTACT_FRICTION 0.58f
#define HG_WALL_TANGENT_KEEP 0.84f
#define HG_RESTITUTION 0.025f
#define HG_SLEEP_SPEED_SQ 4.0f
#define HG_SLEEP_TICKS 8
#define HG_REACTIVATE_IMPACT_SQ 20.0f
#define HG_MAX_RELEASE_PER_STEP 2
#define HG_BOUNDARY_RELAX_BUDGET 24
#define HG_BOUNDARY_NEAR_GAP 3.2f
#define HG_BOUNDARY_SLEEP_GAP 0.65f
#define HG_BOUNDARY_RELAX_STEP 0.22f
#define HG_FLOW_SUSPECT_TICKS 8
#define HG_FLOW_HARD_STALL_TICKS 16
#define HG_FLOW_LOW_FRICTION_TICKS 4
#define HG_FLOW_MIN_UPPER 0
#define HG_THROAT_MIN_ACTIVE 2
#define HG_FLOW_WAKE_BUDGET 3
#define HG_CONTINUITY_INTERVAL_TICKS 4
#define HG_CONTINUITY_WAKE_BUDGET 6
#define HG_UPPER_BAND_COUNT 10
#define HG_UPPER_FEED_ACTIVE_LIMIT 24
#define HG_UPPER_HARD_STALL_ACTIVE_LIMIT 32
#define HG_FLOW_GATE_Y 54.2f
#define HG_STABLE_PILE_RELAX_STEP 0.58f
#define HG_STABLE_ROW_STEP 2.0f

static hg_particle_t s_particles[HG_PARTICLE_CAPACITY];
static int16_t s_grid_head[HG_GRID_CELLS];
static float s_left_boundary[HG_CANVAS_H];
static float s_right_boundary[HG_CANVAS_H];
static float s_left_normal_x[HG_CANVAS_H];
static float s_left_normal_y[HG_CANVAS_H];
static float s_right_normal_x[HG_CANVAS_H];
static float s_right_normal_y[HG_CANVAS_H];
static uint16_t s_particle_count = HG_DEFAULT_PARTICLE_COUNT;
static uint16_t s_requested_particle_count = HG_DEFAULT_PARTICLE_COUNT;
static uint8_t s_max_active = HG_DEFAULT_MAX_ACTIVE;
static uint8_t s_constraint_iterations = HG_DEFAULT_CONSTRAINT_ITERATIONS;
static uint16_t s_last_transferred;
static float s_last_gravity_x;
static uint32_t s_step_counter;
static bool s_all_transferred;
static bool s_allow_upper_reactivation;
static uint16_t s_boundary_relax_cursor;
static uint32_t s_last_particle_pass_step;
static uint16_t s_unblock_count;
static uint8_t s_throat_low_friction_ticks;
static uint8_t s_upper_collision_wake_budget;
static uint8_t s_recovery_passes;
static uint8_t s_flow_stage;
static float s_throat_expansion;
static uint32_t s_last_unblock_step;
static uint32_t s_flow_demand_start_step;
static bool s_flow_demand_active;
static float s_last_progress;
static float s_release_lookahead;
static uint8_t s_flow_pass_quota;
static uint16_t s_stall_detect_count;
static uint16_t s_upper_continuity_fault_count;
static uint16_t s_detected_void_count;
static uint16_t s_bridge_break_count;
static uint16_t s_void_band_mask;
static uint8_t s_upper_surface_active_count;
static uint8_t s_upper_subsurface_active_count;
static bool s_flow_release_due;
static float s_compaction_gravity_x;
static float s_compaction_gravity_y = 1.0f;
static float s_upper_target_x[HG_PARTICLE_CAPACITY];
static float s_upper_target_y[HG_PARTICLE_CAPACITY];
static uint8_t s_upper_target_used[HG_PARTICLE_CAPACITY];
static int16_t s_upper_target_assignment[HG_PARTICLE_CAPACITY];
static uint16_t s_last_compaction_remap_upper;

static float clampf(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static int centered_row_slot(int rank, int count)
{
    int center_left = (count - 1) / 2;
    if (rank == 0) {
        return center_left;
    }
    return (rank & 1) != 0
        ? center_left + (rank + 1) / 2
        : center_left - rank / 2;
}

static float quadratic_x_for_y(float target_y, float x0, float y0,
                               float control_x, float control_y,
                               float x1, float y1)
{
    float best_x = x0;
    float best_error = fabsf(target_y - y0);
    for (int step = 1; step <= 128; ++step) {
        float t = (float)step / 128.0f;
        float one_minus_t = 1.0f - t;
        float y = one_minus_t * one_minus_t * y0 +
                  2.0f * one_minus_t * t * control_y + t * t * y1;
        float error = fabsf(target_y - y);
        if (error < best_error) {
            best_error = error;
            best_x = one_minus_t * one_minus_t * x0 +
                     2.0f * one_minus_t * t * control_x + t * t * x1;
        }
    }
    return best_x;
}

static void init_geometry(void)
{
    for (int y = 0; y < HG_CANVAS_H; ++y) {
        float left = 55.0f;
        float right = 59.0f;
        if (y >= HG_TOP_Y && y <= 27) {
            left = 19.0f;
            right = 95.0f;
        } else if (y > 27 && y <= 53) {
            left = quadratic_x_for_y(
                (float)y, 19.0f, 27.0f, 19.0f, 36.0f, 51.0f, 53.0f);
            right = quadratic_x_for_y(
                (float)y, 95.0f, 27.0f, 95.0f, 36.0f, 63.0f, 53.0f);
        } else if (y > 53 && y <= HG_NECK_TOP_Y + 3) {
            left = quadratic_x_for_y(
                (float)y, 51.0f, 53.0f, 52.5f, 55.0f, 53.5f, 58.0f);
            right = quadratic_x_for_y(
                (float)y, 63.0f, 53.0f, 61.5f, 55.0f, 60.5f, 58.0f);
        } else if (y > 58 && y <= 63) {
            left = quadratic_x_for_y(
                (float)y, 53.5f, 58.0f, 52.5f, 61.0f, 51.0f, 63.0f);
            right = quadratic_x_for_y(
                (float)y, 60.5f, 58.0f, 61.5f, 61.0f, 63.0f, 63.0f);
        } else if (y > 63 && y < 90) {
            left = quadratic_x_for_y(
                (float)y, 51.0f, 63.0f, 19.0f, 80.0f, 19.0f, 89.0f);
            right = quadratic_x_for_y(
                (float)y, 63.0f, 63.0f, 95.0f, 80.0f, 95.0f, 89.0f);
        } else if (y >= 90 && y <= HG_BOTTOM_Y) {
            left = 19.0f;
            right = 95.0f;
        }
        s_left_boundary[y] = left;
        s_right_boundary[y] = right;
    }

    for (int y = 0; y < HG_CANVAS_H; ++y) {
        int previous = y > 0 ? y - 1 : y;
        int next = y + 1 < HG_CANVAS_H ? y + 1 : y;
        float left_slope =
            (s_left_boundary[next] - s_left_boundary[previous]) * 0.5f;
        float right_slope =
            (s_right_boundary[next] - s_right_boundary[previous]) * 0.5f;
        float left_length = sqrtf(1.0f + left_slope * left_slope);
        float right_length = sqrtf(1.0f + right_slope * right_slope);
        s_left_normal_x[y] = 1.0f / left_length;
        s_left_normal_y[y] = -left_slope / left_length;
        s_right_normal_x[y] = -1.0f / right_length;
        s_right_normal_y[y] = right_slope / right_length;
    }
}

static bool in_throat_zone(const hg_particle_t *particle)
{
    return particle->y >= (float)HG_THROAT_ZONE_TOP &&
           particle->y <= (float)HG_THROAT_ZONE_BOTTOM;
}

static bool in_throat_core(const hg_particle_t *particle)
{
    return particle->y >= (float)HG_THROAT_CORE_TOP &&
           particle->y <= (float)HG_THROAT_CORE_BOTTOM;
}

static float throat_expansion_weight(int y)
{
    if (y < HG_THROAT_CORE_TOP || y > HG_THROAT_CORE_BOTTOM) {
        return 0.0f;
    }
    float distance = fabsf((float)y - 58.0f);
    return clampf(1.0f - distance / 9.0f, 0.0f, 1.0f);
}

static float effective_left_boundary(int y)
{
    float timer_expansion =
        clampf((s_release_lookahead - 2.0f) * 0.02f, 0.0f,
               HG_THROAT_MAX_EXPANSION);
    float effective_expansion =
        fmaxf(s_throat_expansion, timer_expansion);
    float expansion =
        HG_THROAT_BASE_WIDTH * effective_expansion *
        throat_expansion_weight(y) * 0.5f;
    return s_left_boundary[y] - expansion;
}

static float effective_right_boundary(int y)
{
    float timer_expansion =
        clampf((s_release_lookahead - 2.0f) * 0.02f, 0.0f,
               HG_THROAT_MAX_EXPANSION);
    float effective_expansion =
        fmaxf(s_throat_expansion, timer_expansion);
    float expansion =
        HG_THROAT_BASE_WIDTH * effective_expansion *
        throat_expansion_weight(y) * 0.5f;
    return s_right_boundary[y] + expansion;
}

static float particle_radius(const hg_particle_t *particle)
{
    return (float)particle->size * 0.5f;
}

static float particle_wall_clearance(const hg_particle_t *particle)
{
    return particle_radius(particle) + 0.5f;
}

static int boundary_row(float y)
{
    int row = (int)(y + 0.5f);
    if (row < 0) {
        return 0;
    }
    if (row >= HG_CANVAS_H) {
        return HG_CANVAS_H - 1;
    }
    return row;
}

float hg_geometry_left_boundary(int y)
{
    if (y < 0) {
        y = 0;
    } else if (y >= HG_CANVAS_H) {
        y = HG_CANVAS_H - 1;
    }
    return effective_left_boundary(y);
}

float hg_geometry_right_boundary(int y)
{
    if (y < 0) {
        y = 0;
    } else if (y >= HG_CANVAS_H) {
        y = HG_CANVAS_H - 1;
    }
    return effective_right_boundary(y);
}

static bool is_active(const hg_particle_t *particle)
{
    return particle->state == HG_PARTICLE_UPPER_ACTIVE ||
           particle->state == HG_PARTICLE_FALLING ||
           particle->state == HG_PARTICLE_LOWER_ACTIVE;
}

static bool is_sleeping(const hg_particle_t *particle)
{
    return particle->state == HG_PARTICLE_UPPER_SLEEPING ||
           particle->state == HG_PARTICLE_LOWER_SLEEPING;
}

static uint16_t active_count(void)
{
    uint16_t count = 0;
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        if (is_active(&s_particles[i])) {
            ++count;
        }
    }
    return count;
}

static int grid_cell(float x, float y)
{
    int column = (int)x / HG_GRID_CELL_SIZE;
    int row = (int)y / HG_GRID_CELL_SIZE;
    if (column < 0) {
        column = 0;
    } else if (column >= HG_GRID_COLS) {
        column = HG_GRID_COLS - 1;
    }
    if (row < 0) {
        row = 0;
    } else if (row >= HG_GRID_ROWS) {
        row = HG_GRID_ROWS - 1;
    }
    return row * HG_GRID_COLS + column;
}

static void rebuild_grid(void)
{
    memset(s_grid_head, 0xff, sizeof(s_grid_head));
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        hg_particle_t *particle = &s_particles[i];
        int cell = grid_cell(particle->x, particle->y);
        particle->grid_next = s_grid_head[cell];
        s_grid_head[cell] = (int16_t)i;
    }
}

static uint8_t support_count_below(uint16_t index)
{
    const hg_particle_t *particle = &s_particles[index];
    uint8_t supports = 0;
    int cell = grid_cell(particle->x, particle->y);
    int base_row = cell / HG_GRID_COLS;
    int base_column = cell % HG_GRID_COLS;
    for (int row_offset = 0; row_offset <= 1; ++row_offset) {
        int row = base_row + row_offset;
        if (row >= HG_GRID_ROWS) {
            continue;
        }
        for (int column_offset = -1; column_offset <= 1; ++column_offset) {
            int column = base_column + column_offset;
            if (column < 0 || column >= HG_GRID_COLS) {
                continue;
            }
            int16_t other_index = s_grid_head[row * HG_GRID_COLS + column];
            while (other_index >= 0) {
                if ((uint16_t)other_index != index) {
                    const hg_particle_t *other = &s_particles[other_index];
                    float dx = other->x - particle->x;
                    float dy = other->y - particle->y;
                    if (dy > 0.8f && dy < 3.4f && dx * dx < 8.0f) {
                        if (++supports >= 3) {
                            return supports;
                        }
                    }
                }
                other_index = s_particles[other_index].grid_next;
            }
        }
    }
    if (particle->y >= (float)HG_BOTTOM_Y - 1.5f) {
        return 3;
    }
    return supports;
}

static bool has_support_below(uint16_t index)
{
    return support_count_below(index) > 0;
}

static uint8_t particle_count_above(uint16_t index)
{
    const hg_particle_t *particle = &s_particles[index];
    uint8_t above = 0;
    int cell = grid_cell(particle->x, particle->y);
    int base_row = cell / HG_GRID_COLS;
    int base_column = cell % HG_GRID_COLS;
    for (int row_offset = -1; row_offset <= 0; ++row_offset) {
        int row = base_row + row_offset;
        if (row < 0) {
            continue;
        }
        for (int column_offset = -1; column_offset <= 1; ++column_offset) {
            int column = base_column + column_offset;
            if (column < 0 || column >= HG_GRID_COLS) {
                continue;
            }
            int16_t other_index = s_grid_head[row * HG_GRID_COLS + column];
            while (other_index >= 0) {
                if ((uint16_t)other_index != index) {
                    const hg_particle_t *other = &s_particles[other_index];
                    float dx = other->x - particle->x;
                    float dy = other->y - particle->y;
                    if (dy < -0.8f && dy > -3.8f && dx * dx < 9.0f) {
                        if (++above >= 3) {
                            return above;
                        }
                    }
                }
                other_index = s_particles[other_index].grid_next;
            }
        }
    }
    return above;
}

static bool has_particle_above(uint16_t index)
{
    return particle_count_above(index) > 0;
}

static uint8_t nearby_particle_count(uint16_t index)
{
    const hg_particle_t *particle = &s_particles[index];
    int cell = grid_cell(particle->x, particle->y);
    int base_row = cell / HG_GRID_COLS;
    int base_column = cell % HG_GRID_COLS;
    uint8_t neighbors = 0;
    for (int row_offset = -1; row_offset <= 1; ++row_offset) {
        int row = base_row + row_offset;
        if (row < 0 || row >= HG_GRID_ROWS) {
            continue;
        }
        for (int column_offset = -1; column_offset <= 1; ++column_offset) {
            int column = base_column + column_offset;
            if (column < 0 || column >= HG_GRID_COLS) {
                continue;
            }
            int16_t other_index =
                s_grid_head[row * HG_GRID_COLS + column];
            while (other_index >= 0) {
                if ((uint16_t)other_index != index) {
                    const hg_particle_t *other = &s_particles[other_index];
                    float dx = other->x - particle->x;
                    float dy = other->y - particle->y;
                    if (dx * dx + dy * dy < 15.0f) {
                        if (++neighbors >= 8) {
                            return neighbors;
                        }
                    }
                }
                other_index = s_particles[other_index].grid_next;
            }
        }
    }
    return neighbors;
}

static void wake_particle(uint16_t index, hg_particle_state_t state)
{
    if (index >= s_particle_count || active_count() >= s_max_active) {
        return;
    }
    hg_particle_t *particle = &s_particles[index];
    if (!is_sleeping(particle)) {
        return;
    }
    particle->state = (uint8_t)state;
    particle->stable_ticks = 0;
}

static uint16_t transferred_count(void)
{
    uint16_t count = 0;
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        if (s_particles[i].state == HG_PARTICLE_FALLING ||
            s_particles[i].state == HG_PARTICLE_LOWER_ACTIVE ||
            s_particles[i].state == HG_PARTICLE_LOWER_SLEEPING) {
            ++count;
        }
    }
    return count;
}

static uint8_t feed_active_count(void)
{
    uint8_t count = 0;
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        if (s_particles[i].state == HG_PARTICLE_UPPER_ACTIVE ||
            s_particles[i].state == HG_PARTICLE_FALLING) {
            ++count;
        }
    }
    return count;
}

static void wake_for_target(float progress, bool allow_flow)
{
    if (!allow_flow) {
        return;
    }
    uint16_t transferred = transferred_count();
    uint16_t target = (uint16_t)clampf(
        floorf(progress * (float)s_particle_count +
               s_release_lookahead),
        0.0f, (float)s_particle_count);
    uint16_t feed_active_limit = (uint16_t)s_max_active + 12u;
    if (feed_active_limit > 100u) {
        feed_active_limit = 100u;
    }
    if (target <= transferred || active_count() >= feed_active_limit) {
        return;
    }

    uint16_t deficit = target - transferred;
    uint8_t feed_active = feed_active_count();
    uint8_t timer_feed_limit =
        s_release_lookahead > 2.0f
        ? HG_UPPER_HARD_STALL_ACTIVE_LIMIT
        : HG_UPPER_FEED_ACTIVE_LIMIT;
    uint8_t desired_feed =
        deficit + 4u > timer_feed_limit
        ? timer_feed_limit
        : (uint8_t)(deficit + 4u);
    if (feed_active >= desired_feed) {
        return;
    }
    uint16_t release_count =
        deficit > HG_MAX_RELEASE_PER_STEP ? HG_MAX_RELEASE_PER_STEP : deficit;
    if (release_count > desired_feed - feed_active) {
        release_count = desired_feed - feed_active;
    }
    for (uint16_t release = 0; release < release_count; ++release) {
        int best_index = -1;
        float best_score = -100000.0f;
        for (uint16_t i = 0; i < s_particle_count; ++i) {
            hg_particle_t *particle = &s_particles[i];
            if (particle->state != HG_PARTICLE_UPPER_SLEEPING) {
                continue;
            }
            float center_distance = fabsf(particle->x - HG_CENTER_X);
            float score = particle->y * 5.0f - center_distance * 2.2f;
            if (!has_support_below(i)) {
                score += 18.0f;
            }
            if (score > best_score) {
                best_score = score;
                best_index = (int)i;
            }
        }
        if (best_index < 0) {
            break;
        }
        s_particles[best_index].state = HG_PARTICLE_UPPER_ACTIVE;
        s_particles[best_index].stable_ticks = 0;
        s_particles[best_index].vy += 0.8f;
    }
}

static void wake_funnel_and_slopes(float gravity_x, float progress)
{
    uint16_t available =
        active_count() < s_max_active ? s_max_active - active_count() : 0;
    if (available == 0) {
        return;
    }

    uint16_t transferred = transferred_count();
    uint16_t target = (uint16_t)clampf(
        floorf(progress * (float)s_particle_count +
               s_release_lookahead),
        0.0f, (float)s_particle_count);
    uint16_t active_upper = feed_active_count();
    uint16_t desired_upper =
        target > transferred ? target - transferred + 2u : 0u;
    uint16_t timer_feed_limit =
        s_release_lookahead > 2.0f
        ? HG_UPPER_HARD_STALL_ACTIVE_LIMIT
        : HG_UPPER_FEED_ACTIVE_LIMIT;
    if (desired_upper > timer_feed_limit) {
        desired_upper = timer_feed_limit;
    }
    uint16_t funnel_budget =
        desired_upper > active_upper ? desired_upper - active_upper : 0u;
    if (funnel_budget > HG_CONTINUITY_WAKE_BUDGET) {
        funnel_budget = HG_CONTINUITY_WAKE_BUDGET;
    }

    float funnel_radius = 5.0f + progress * 22.0f;
    for (uint16_t i = 0;
         i < s_particle_count && available > 0 && funnel_budget > 0;
         ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (particle->state == HG_PARTICLE_UPPER_SLEEPING &&
            fabsf(particle->x - HG_CENTER_X) < funnel_radius &&
            !has_support_below(i)) {
            wake_particle(i, HG_PARTICLE_UPPER_ACTIVE);
            --available;
            --funnel_budget;
        }
    }

    if (fabsf(gravity_x) > 0.16f && available > 0) {
        for (uint16_t i = 0; i < s_particle_count && available > 0; ++i) {
            hg_particle_t *particle = &s_particles[i];
            if (particle->state != HG_PARTICLE_LOWER_SLEEPING ||
                has_particle_above(i)) {
                continue;
            }
            bool uphill_side = gravity_x > 0.0f
                ? particle->x < HG_CENTER_X
                : particle->x > HG_CENTER_X;
            if (uphill_side && ((i + s_step_counter) % 5u == 0u)) {
                wake_particle(i, HG_PARTICLE_LOWER_ACTIVE);
                --available;
            }
        }
    }
}

static void keep_wall_tangent_velocity(hg_particle_t *particle,
                                       float inward_normal_x,
                                       float inward_normal_y)
{
    /*
     * The position correction already removed wall penetration. Retaining an
     * inward normal component here would look like a bounce away from the
     * glass, so keep only the damped tangent component.
     */
    float tangent_x = -inward_normal_y;
    float tangent_y = inward_normal_x;
    float tangent_speed =
        particle->vx * tangent_x + particle->vy * tangent_y;
    tangent_speed *= HG_WALL_TANGENT_KEEP;
    particle->vx = tangent_x * tangent_speed;
    particle->vy = tangent_y * tangent_speed;
}

static void constrain_to_glass(hg_particle_t *particle)
{
    float clearance = particle_wall_clearance(particle);
    float top = (float)HG_TOP_Y + clearance;
    float bottom = (float)HG_BOTTOM_Y - clearance;
    if (particle->y < top) {
        particle->y = top;
        if (particle->vy < 0.0f) {
            particle->vy = 0.0f;
        }
    } else if (particle->y > bottom) {
        particle->y = bottom;
        if (particle->vy > 0.0f) {
            particle->vy = 0.0f;
        }
        particle->vx *= HG_CONTACT_FRICTION;
    }
    int y = boundary_row(particle->y);
    float left = effective_left_boundary(y) + clearance;
    float right = effective_right_boundary(y) - clearance;
    if (particle->x < left) {
        particle->x = left;
        keep_wall_tangent_velocity(
            particle, s_left_normal_x[y], s_left_normal_y[y]);
    } else if (particle->x > right) {
        particle->x = right;
        keep_wall_tangent_velocity(
            particle, s_right_normal_x[y], s_right_normal_y[y]);
    }
}

static void integrate_particles(float gravity_x, float gravity_y)
{
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (!is_active(particle)) {
            continue;
        }
        float local_gravity =
            in_throat_zone(particle) ? HG_GRAVITY_ACCEL * 2.10f
                                     : HG_GRAVITY_ACCEL;
        if (s_flow_release_due &&
            particle->state == HG_PARTICLE_UPPER_ACTIVE) {
            float fast_timer_scale =
                1.0f +
                clampf(s_release_lookahead - 1.0f, 0.0f, 4.0f) *
                    0.22f;
            local_gravity *= fast_timer_scale;
        }
        float local_gravity_x = gravity_x;
        if (particle->y >= (float)HG_THROAT_ZONE_TOP &&
            particle->y < (float)HG_NECK_TOP_Y) {
            /*
             * The converging glass guides grains toward the outlet even when
             * the device is tilted. Preserve IMU direction, but attenuate its
             * lateral component only inside this short supply funnel.
             */
            local_gravity_x *= 0.25f;
        }
        particle->vx =
            (particle->vx +
             local_gravity_x * local_gravity * HG_PHYSICS_DT) *
            HG_AIR_DAMPING;
        if (s_flow_release_due &&
            particle->state == HG_PARTICLE_UPPER_ACTIVE &&
            particle->y >= 20.0f &&
            particle->y < (float)HG_NECK_TOP_Y) {
            float center_direction =
                clampf((HG_CENTER_X - particle->x) * 0.10f, -1.0f, 1.0f);
            particle->vx +=
                center_direction * HG_GRAVITY_ACCEL * 0.62f *
                HG_PHYSICS_DT;
        }
        particle->vy =
            (particle->vy + gravity_y * local_gravity * HG_PHYSICS_DT) *
            HG_AIR_DAMPING;
        particle->vx = clampf(particle->vx, -HG_MAX_SPEED, HG_MAX_SPEED);
        particle->vy = clampf(particle->vy, -HG_MAX_SPEED, HG_MAX_SPEED);
        float previous_y = particle->y;
        particle->x += particle->vx * HG_PHYSICS_DT;
        particle->y += particle->vy * HG_PHYSICS_DT;
        /*
         * Surface and subsurface grains stay physically alive while progress
         * is momentarily caught up.  This narrow non-bouncing gate meters
         * real grains above the visible throat; it never transfers, deletes,
         * or recreates a particle.
         */
        if (s_flow_pass_quota == 0 &&
            particle->state == HG_PARTICLE_UPPER_ACTIVE &&
            particle->y > HG_FLOW_GATE_Y) {
            particle->y = HG_FLOW_GATE_Y;
            if (particle->vy > 0.0f) {
                particle->vy = 0.0f;
            }
            particle->vx *= 0.72f;
        }
        constrain_to_glass(particle);
        if (previous_y <= (float)HG_NECK_BOTTOM_Y + 1.0f &&
            particle->y > (float)HG_NECK_BOTTOM_Y + 1.0f) {
            s_last_particle_pass_step = s_step_counter;
            s_flow_stage = 0;
            if (s_throat_expansion > 0.0f && s_recovery_passes < 255) {
                ++s_recovery_passes;
            }
        }
    }
}

static void solve_particle_collisions(void)
{
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (!is_active(particle)) {
            continue;
        }
        int cell = grid_cell(particle->x, particle->y);
        int base_row = cell / HG_GRID_COLS;
        int base_column = cell % HG_GRID_COLS;
        for (int row_offset = -1; row_offset <= 1; ++row_offset) {
            int row = base_row + row_offset;
            if (row < 0 || row >= HG_GRID_ROWS) {
                continue;
            }
            for (int column_offset = -1; column_offset <= 1; ++column_offset) {
                int column = base_column + column_offset;
                if (column < 0 || column >= HG_GRID_COLS) {
                    continue;
                }
                int16_t other_index =
                    s_grid_head[row * HG_GRID_COLS + column];
                while (other_index >= 0) {
                    if ((uint16_t)other_index != i) {
                        hg_particle_t *other = &s_particles[other_index];
                        if (is_active(other) && (uint16_t)other_index < i) {
                            other_index = other->grid_next;
                            continue;
                        }
                        float dx = particle->x - other->x;
                        float dy = particle->y - other->y;
                        float minimum_distance =
                            particle_radius(particle) +
                            particle_radius(other);
                        float distance_sq = dx * dx + dy * dy;
                        float minimum_sq =
                            minimum_distance * minimum_distance;
                        if (distance_sq < minimum_sq) {
                            float distance =
                                distance_sq > 0.01f
                                ? sqrtf(distance_sq)
                                : 0.0f;
                            float nx =
                                distance > 0.0f
                                ? dx / distance
                                : (((i + (uint16_t)other_index) & 1u)
                                       ? -0.7071f
                                       : 0.7071f);
                            float ny =
                                distance > 0.0f ? dy / distance : -0.7071f;
                            float overlap = minimum_distance - distance;
                            bool throat_contact =
                                in_throat_zone(particle) ||
                                in_throat_zone(other);
                            if (throat_contact && overlap > 0.55f) {
                                overlap = 0.55f;
                            }
                            bool other_moves = is_active(other);
                            float particle_share = other_moves ? 0.5f : 1.0f;
                            particle->x += nx * overlap * particle_share;
                            particle->y += ny * overlap * particle_share;
                            if (other_moves) {
                                other->x -= nx * overlap * 0.5f;
                                other->y -= ny * overlap * 0.5f;
                            }

                            float relative_vx = particle->vx - other->vx;
                            float relative_vy = particle->vy - other->vy;
                            float normal_speed =
                                relative_vx * nx + relative_vy * ny;
                            if (normal_speed < 0.0f) {
                                float impulse =
                                    -(1.0f +
                                      (throat_contact ? 0.0f : HG_RESTITUTION)) *
                                    normal_speed;
                                particle->vx += nx * impulse * particle_share;
                                particle->vy += ny * impulse * particle_share;
                                float velocity_keep = throat_contact
                                    ? (s_throat_low_friction_ticks > 0
                                           ? 0.94f
                                           : 0.86f)
                                    : HG_CONTACT_FRICTION;
                                particle->vx *= velocity_keep;
                                particle->vy *=
                                    0.82f + velocity_keep * 0.18f;
                            }

                            bool upper_sleeping =
                                other->state == HG_PARTICLE_UPPER_SLEEPING;
                            bool upper_wake_allowed =
                                !upper_sleeping ||
                                (s_allow_upper_reactivation &&
                                 s_upper_collision_wake_budget > 0);
                            if (!s_all_transferred && is_sleeping(other) &&
                                upper_wake_allowed &&
                                relative_vx * relative_vx +
                                        relative_vy * relative_vy >
                                    HG_REACTIVATE_IMPACT_SQ &&
                                active_count() < s_max_active) {
                                wake_particle(
                                    (uint16_t)other_index,
                                    other->state ==
                                            HG_PARTICLE_UPPER_SLEEPING
                                        ? HG_PARTICLE_UPPER_ACTIVE
                                        : HG_PARTICLE_LOWER_ACTIVE);
                                if (upper_sleeping &&
                                    s_upper_collision_wake_budget > 0) {
                                    --s_upper_collision_wake_budget;
                                }
                            }
                        }
                    }
                    other_index = s_particles[other_index].grid_next;
                }
            }
        }
        constrain_to_glass(particle);
    }
}

static void solve_throat_collisions(void)
{
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (!is_active(particle) || !in_throat_zone(particle)) {
            continue;
        }
        int cell = grid_cell(particle->x, particle->y);
        int base_row = cell / HG_GRID_COLS;
        int base_column = cell % HG_GRID_COLS;
        for (int row_offset = -1; row_offset <= 1; ++row_offset) {
            int row = base_row + row_offset;
            if (row < 0 || row >= HG_GRID_ROWS) {
                continue;
            }
            for (int column_offset = -1; column_offset <= 1; ++column_offset) {
                int column = base_column + column_offset;
                if (column < 0 || column >= HG_GRID_COLS) {
                    continue;
                }
                int16_t other_index =
                    s_grid_head[row * HG_GRID_COLS + column];
                while (other_index >= 0) {
                    hg_particle_t *other = &s_particles[other_index];
                    if ((uint16_t)other_index > i && is_active(other) &&
                        in_throat_zone(other)) {
                        float dx = particle->x - other->x;
                        float dy = particle->y - other->y;
                        float minimum =
                            particle_radius(particle) +
                            particle_radius(other);
                        float distance_sq = dx * dx + dy * dy;
                        if (distance_sq < minimum * minimum) {
                            float distance =
                                distance_sq > 0.01f
                                ? sqrtf(distance_sq)
                                : 0.0f;
                            float correction =
                                fminf((minimum - distance) * 0.5f, 0.28f);
                            float nx =
                                distance > 0.0f
                                ? dx / distance
                                : (((i + (uint16_t)other_index) & 1u)
                                       ? -0.7071f
                                       : 0.7071f);
                            float ny =
                                distance > 0.0f ? dy / distance : -0.7071f;
                            particle->x += nx * correction;
                            particle->y += ny * correction;
                            other->x -= nx * correction;
                            other->y -= ny * correction;
                            constrain_to_glass(particle);
                            constrain_to_glass(other);
                        }
                    }
                    other_index = other->grid_next;
                }
            }
        }
    }
}

static void enforce_flow_gate(void)
{
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (particle->state != HG_PARTICLE_UPPER_ACTIVE ||
            particle->y <= HG_FLOW_GATE_Y) {
            continue;
        }
        if (s_flow_pass_quota > 0) {
            particle->state = HG_PARTICLE_FALLING;
            particle->stable_ticks = 0;
            --s_flow_pass_quota;
            continue;
        }
        particle->y = HG_FLOW_GATE_Y;
        if (particle->vy > 0.0f) {
            particle->vy = 0.0f;
        }
        particle->vx *= 0.72f;
        constrain_to_glass(particle);
    }
    rebuild_grid();
}

static void update_regions_and_sleep(void)
{
    rebuild_grid();
    bool all_transferred = transferred_count() == s_particle_count;
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (!is_active(particle)) {
            continue;
        }
        if (particle->state == HG_PARTICLE_FALLING &&
            particle->y >= HG_FLOW_GATE_Y &&
            particle->y < (float)HG_NECK_BOTTOM_Y + 5.0f) {
            particle->stable_ticks = 0;
        } else if (particle->y > (float)HG_NECK_TOP_Y &&
            particle->y < (float)HG_NECK_BOTTOM_Y + 5.0f) {
            particle->state = HG_PARTICLE_FALLING;
            particle->stable_ticks = 0;
        } else if (particle->y >= (float)HG_NECK_BOTTOM_Y + 5.0f) {
            particle->state = HG_PARTICLE_LOWER_ACTIVE;
        } else if (particle->state != HG_PARTICLE_UPPER_ACTIVE) {
            particle->state = HG_PARTICLE_UPPER_ACTIVE;
        }
        if (all_transferred &&
            particle->state == HG_PARTICLE_LOWER_ACTIVE) {
            particle->vx *= 0.45f;
            particle->vy *= 0.45f;
        }

        float speed_sq =
            particle->vx * particle->vx + particle->vy * particle->vy;
        int boundary_y = boundary_row(particle->y);
        float clearance = particle_wall_clearance(particle);
        float left_gap =
            particle->x -
            (effective_left_boundary(boundary_y) + clearance);
        float right_gap =
            (effective_right_boundary(boundary_y) - clearance) - particle->x;
        float wall_gap = fminf(left_gap, right_gap);
        bool near_wall = wall_gap < HG_BOUNDARY_NEAR_GAP;
        bool wall_aligned =
            !near_wall || wall_gap <= HG_BOUNDARY_SLEEP_GAP;
        bool wall_support =
            particle->state == HG_PARTICLE_LOWER_ACTIVE &&
            wall_gap <= HG_BOUNDARY_SLEEP_GAP;
        bool upper_particle =
            particle->state == HG_PARTICLE_UPPER_ACTIVE;
        uint8_t support_count =
            upper_particle ? support_count_below(i) : 0;
        uint8_t above_count =
            upper_particle ? particle_count_above(i) : 3;
        uint8_t neighbor_count =
            upper_particle ? nearby_particle_count(i) : 4;
        float supply_half_width =
            20.0f + fmaxf(0.0f, 55.0f - particle->y) * 0.70f;
        bool upper_supply_zone =
            upper_particle && particle->y >= 24.0f &&
            fabsf(particle->x - HG_CENTER_X) < supply_half_width;
        bool upper_surface_or_subsurface =
            upper_particle && above_count <= 2;
        bool stable_upper_support =
            !upper_particle ||
            (support_count >= 2 && neighbor_count >= 3 &&
             !upper_supply_zone && !upper_surface_or_subsurface);
        bool can_sleep =
            particle->state != HG_PARTICLE_FALLING &&
            !in_throat_zone(particle) &&
            speed_sq < HG_SLEEP_SPEED_SQ &&
            wall_aligned &&
            stable_upper_support &&
            (particle->state == HG_PARTICLE_LOWER_ACTIVE ||
             has_support_below(i) || wall_support);
        if (can_sleep) {
            if (particle->stable_ticks < 255) {
                ++particle->stable_ticks;
            }
        } else {
            particle->stable_ticks = 0;
        }
        if (particle->stable_ticks >= HG_SLEEP_TICKS) {
            particle->vx = 0.0f;
            particle->vy = 0.0f;
            particle->state =
                particle->y < (float)HG_NECK_TOP_Y
                ? HG_PARTICLE_UPPER_SLEEPING
                : HG_PARTICLE_LOWER_SLEEPING;
        }

        float blend =
            (particle->y - (float)HG_NECK_TOP_Y) / 18.0f;
        particle->color_phase =
            (uint8_t)(clampf(blend, 0.0f, 1.0f) * 4.0f + 0.5f);
    }
}

static bool position_overlaps_neighbor(uint16_t index, float x, float y)
{
    const hg_particle_t *particle = &s_particles[index];
    int cell = grid_cell(x, y);
    int base_row = cell / HG_GRID_COLS;
    int base_column = cell % HG_GRID_COLS;
    for (int row_offset = -1; row_offset <= 1; ++row_offset) {
        int row = base_row + row_offset;
        if (row < 0 || row >= HG_GRID_ROWS) {
            continue;
        }
        for (int column_offset = -1; column_offset <= 1; ++column_offset) {
            int column = base_column + column_offset;
            if (column < 0 || column >= HG_GRID_COLS) {
                continue;
            }
            int16_t other_index =
                s_grid_head[row * HG_GRID_COLS + column];
            while (other_index >= 0) {
                if ((uint16_t)other_index != index) {
                    const hg_particle_t *other = &s_particles[other_index];
                    float dx = x - other->x;
                    float dy = y - other->y;
                    float minimum =
                        (particle_radius(particle) + particle_radius(other)) *
                        0.92f;
                    if (dx * dx + dy * dy < minimum * minimum) {
                        return true;
                    }
                }
                other_index = s_particles[other_index].grid_next;
            }
        }
    }
    return false;
}

static bool in_upper_supply_zone(const hg_particle_t *particle)
{
    if (particle->y < 20.0f || particle->y >= (float)HG_NECK_TOP_Y) {
        return false;
    }
    float half_width =
        20.0f + fmaxf(0.0f, 55.0f - particle->y) * 0.70f;
    return fabsf(particle->x - HG_CENTER_X) < half_width;
}

static void solve_upper_funnel_collisions(void)
{
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (!is_active(particle) || !in_upper_supply_zone(particle)) {
            continue;
        }
        int cell = grid_cell(particle->x, particle->y);
        int base_row = cell / HG_GRID_COLS;
        int base_column = cell % HG_GRID_COLS;
        for (int row_offset = -1; row_offset <= 1; ++row_offset) {
            int row = base_row + row_offset;
            if (row < 0 || row >= HG_GRID_ROWS) {
                continue;
            }
            for (int column_offset = -1; column_offset <= 1; ++column_offset) {
                int column = base_column + column_offset;
                if (column < 0 || column >= HG_GRID_COLS) {
                    continue;
                }
                int16_t other_index =
                    s_grid_head[row * HG_GRID_COLS + column];
                while (other_index >= 0) {
                    hg_particle_t *other = &s_particles[other_index];
                    if ((uint16_t)other_index > i && is_active(other) &&
                        in_upper_supply_zone(other)) {
                        float dx = particle->x - other->x;
                        float dy = particle->y - other->y;
                        float minimum =
                            particle_radius(particle) +
                            particle_radius(other);
                        float distance_sq = dx * dx + dy * dy;
                        if (distance_sq < minimum * minimum) {
                            float distance =
                                distance_sq > 0.01f
                                ? sqrtf(distance_sq)
                                : 0.0f;
                            float correction =
                                fminf((minimum - distance) * 0.5f, 0.20f);
                            float nx =
                                distance > 0.0f
                                ? dx / distance
                                : (((i + (uint16_t)other_index) & 1u)
                                       ? -0.7071f
                                       : 0.7071f);
                            float ny =
                                distance > 0.0f ? dy / distance : -0.7071f;
                            particle->x += nx * correction;
                            particle->y += ny * correction;
                            other->x -= nx * correction;
                            other->y -= ny * correction;
                            constrain_to_glass(particle);
                            constrain_to_glass(other);
                        }
                    }
                    other_index = other->grid_next;
                }
            }
        }
    }
}

static bool relax_upper_stable_pile(void)
{
    /*
     * The sleeping grains are the precomputed stable-pile half of the hybrid
     * model.  Reserve the lowest compact slots for the physically active
     * surface/core grains, then let sleeping grains converge slowly toward
     * the remaining slots.  This maintains volume and closes unsupported
     * horizontal bands without turning all 300 grains into active bodies.
     */
    uint16_t upper_active = 0;
    uint16_t upper_sleeping = 0;
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        if (s_particles[i].state == HG_PARTICLE_UPPER_ACTIVE) {
            ++upper_active;
        } else if (s_particles[i].state == HG_PARTICLE_UPPER_SLEEPING) {
            ++upper_sleeping;
        }
    }
    if (upper_sleeping == 0) {
        return false;
    }

    /*
     * Only expose as many compact slots as there are grains still in the
     * upper chamber. As grains transfer, the highest slots disappear and
     * cached sleepers are forced to remap downward. Keeping all 300 slots
     * available allowed remaining grains to retain their old high positions,
     * leaving a suspended horizontal slab above an empty funnel.
     */
    uint16_t upper_target_limit = upper_sleeping + upper_active;
    uint16_t target_count = 0;
    int row = 0;
    for (float target_y = 53.0f;
         target_y >= 11.0f && target_count < upper_target_limit;
         target_y -= HG_STABLE_ROW_STEP, ++row) {
        int boundary_y = boundary_row(target_y);
        float spacing = 2.90f;
        float start =
            s_left_boundary[boundary_y] + 1.5f +
            (row & 1 ? spacing * 0.5f : 0.0f);
        float right = s_right_boundary[boundary_y] - 1.5f;
        int row_slots = (int)((right - start) / spacing) + 1;
        for (int rank = 0;
             rank < row_slots && target_count < upper_target_limit;
             ++rank) {
            float target_x =
                start +
                (float)centered_row_slot(rank, row_slots) * spacing;
            /*
             * A tiny deterministic stagger keeps the sleeping half of the
             * hybrid pile dense without exposing mechanical scan-line rows.
             * It is stable frame-to-frame, so there is no shimmer and no
             * extra live physics cost on the ESP32-C6.
             */
            int jitter_seed = (int)target_count * 37 + row * 11;
            float jitter_x =
                (float)((jitter_seed % 7) - 3) * 0.09f;
            float jitter_y =
                (float)((((int)target_count * 17 + row * 5) % 5) - 2) *
                0.08f;
            target_x += jitter_x;
            float gravity_shift =
                s_compaction_gravity_x * (55.0f - target_y) * 0.34f;
            float shifted_x = target_x + gravity_shift;
            float shifted_y =
                target_y + jitter_y -
                s_compaction_gravity_x *
                    (shifted_x - HG_CENTER_X) * 0.24f;
            s_upper_target_x[target_count] = shifted_x;
            s_upper_target_y[target_count] = shifted_y;
            s_upper_target_used[target_count] = 0;
            ++target_count;
        }
    }

    /*
     * Reserve only the stable-pile slots that active upper grains actually
     * occupy.  The previous fixed reservation removed the first N compact
     * slots, which are the center slots on successive rows; as soon as the
     * timer started it therefore carved a visible vertical crack through the
     * upper pile.  Nearest-slot reservation follows the real moving surface
     * and releases a slot again as soon as its active grain moves away.
     */
    for (uint16_t i = 0; i < s_particle_count && upper_active > 0; ++i) {
        const hg_particle_t *particle = &s_particles[i];
        if (particle->state != HG_PARTICLE_UPPER_ACTIVE) {
            continue;
        }
        int best_target = -1;
        float best_distance_sq = 16.0f;
        for (uint16_t target = 0; target < target_count; ++target) {
            if (s_upper_target_used[target] != 0) {
                continue;
            }
            float dx = s_upper_target_x[target] - particle->x;
            float dy = s_upper_target_y[target] - particle->y;
            float distance_sq = dx * dx + dy * dy;
            if (distance_sq < best_distance_sq) {
                best_distance_sq = distance_sq;
                best_target = (int)target;
            }
        }
        if (best_target >= 0) {
            s_upper_target_used[best_target] = 1;
        }
    }

    /*
     * A one-minute timer can release several bottom grains before the cached
     * target chain has naturally cascaded. During the first quarter, remap
     * after every small reduction in upper volume so nearby grains fill the
     * newly vacant funnel slots instead of waiting for far-away top grains.
     * Later in the run the stable cache takes over again.
     */
    bool early_compaction = upper_target_limit >
        (uint16_t)(s_particle_count * 3u / 4u);
    if (early_compaction &&
        s_last_compaction_remap_upper > upper_target_limit + 2u) {
        memset(s_upper_target_assignment, 0xff,
               sizeof(s_upper_target_assignment));
        s_last_compaction_remap_upper = upper_target_limit;
    } else if (!early_compaction) {
        s_last_compaction_remap_upper = upper_target_limit;
    }

    /*
     * Target order is stable between continuity passes. Keep each sleeping
     * grain on its previous nearby slot and only run the full nearest-slot
     * search for newly sleeping grains or invalidated assignments. This
     * preserves the exact target geometry while removing the usual O(N^2)
     * matching cost.
     */
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (particle->state != HG_PARTICLE_UPPER_SLEEPING) {
            s_upper_target_assignment[i] = -1;
            continue;
        }
        int assigned = s_upper_target_assignment[i];
        if (assigned < 0 || assigned >= target_count ||
            s_upper_target_used[assigned] != 0) {
            s_upper_target_assignment[i] = -1;
            continue;
        }
        float dx = s_upper_target_x[assigned] - particle->x;
        float dy = s_upper_target_y[assigned] - particle->y;
        if (dx * dx + dy * dy > 64.0f) {
            s_upper_target_assignment[i] = -1;
            continue;
        }
        s_upper_target_used[assigned] = 1;
    }

    bool moved = false;
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (particle->state != HG_PARTICLE_UPPER_SLEEPING) {
            continue;
        }
        int best_target = s_upper_target_assignment[i];
        if (best_target < 0) {
            float best_distance_sq = 1000000.0f;
            for (uint16_t target = 0; target < target_count; ++target) {
                if (s_upper_target_used[target] != 0) {
                    continue;
                }
                float dx = s_upper_target_x[target] - particle->x;
                float dy = s_upper_target_y[target] - particle->y;
                float distance_sq = dx * dx + dy * dy;
                if (distance_sq < best_distance_sq) {
                    best_distance_sq = distance_sq;
                    best_target = (int)target;
                }
            }
            if (best_target < 0) {
                break;
            }
            s_upper_target_assignment[i] = (int16_t)best_target;
            s_upper_target_used[best_target] = 1;
        }
        float shifted_y = s_upper_target_y[best_target];
        int shifted_row = boundary_row(shifted_y);
        float clearance = particle_wall_clearance(particle);
        float shifted_x = clampf(
            s_upper_target_x[best_target],
            effective_left_boundary(shifted_row) + clearance,
            effective_right_boundary(shifted_row) - clearance);
        float dx = shifted_x - particle->x;
        float dy = shifted_y - particle->y;
            /*
             * With the device nearly upright, stable grains settle down and
             * sideways only.  Under real tilt they may also climb the screen
             * along a glass wall, provided the move remains downhill in the
             * measured gravity field.
             */
        float downhill =
            dx * s_compaction_gravity_x +
            dy * s_compaction_gravity_y;
        if (dy < 0.0f && fabsf(s_compaction_gravity_x) < 0.12f) {
            dy = 0.0f;
        } else if (downhill < 0.0f) {
            dx -= downhill * s_compaction_gravity_x;
            dy -= downhill * s_compaction_gravity_y;
        }
        float distance = sqrtf(dx * dx + dy * dy);
        if (distance > 0.01f) {
            float relax_step =
                early_compaction ? 0.90f : HG_STABLE_PILE_RELAX_STEP;
            float scale =
                fminf(relax_step, distance) / distance;
            particle->x += dx * scale;
            particle->y += dy * scale;
            constrain_to_glass(particle);
            particle->stable_ticks = 0;
            moved = true;
        }
    }
    return moved;
}

static void upper_layer_continuity_check(bool force)
{
    if (!force &&
        (s_step_counter % HG_CONTINUITY_INTERVAL_TICKS) != 0u) {
        return;
    }
    rebuild_grid();
    uint8_t band_counts[HG_UPPER_BAND_COUNT] = {0};
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        const hg_particle_t *particle = &s_particles[i];
        if ((particle->state == HG_PARTICLE_UPPER_SLEEPING ||
             particle->state == HG_PARTICLE_UPPER_ACTIVE) &&
            particle->y >= 11.0f && particle->y < 55.0f) {
            int band = (int)((particle->y - 11.0f) / 4.4f);
            if (band >= 0 && band < HG_UPPER_BAND_COUNT &&
                band_counts[band] < 255) {
                ++band_counts[band];
            }
        }
    }

    uint16_t new_void_mask = 0;
    for (int band = 1; band < HG_UPPER_BAND_COUNT - 1; ++band) {
        if (band_counts[band] <= 2 &&
            band_counts[band - 1] >= 5 &&
            band_counts[band + 1] >= 5) {
            new_void_mask |= (uint16_t)(1u << band);
        }
    }
    uint16_t new_faults = new_void_mask & (uint16_t)~s_void_band_mask;
    if (new_faults != 0) {
        for (int band = 0; band < HG_UPPER_BAND_COUNT; ++band) {
            if ((new_faults & (uint16_t)(1u << band)) != 0) {
                ++s_detected_void_count;
                ++s_upper_continuity_fault_count;
            }
        }
    }
    s_void_band_mask = new_void_mask;

    uint16_t active_limit = (uint16_t)s_max_active + 12u;
    if (active_limit > 100u) {
        active_limit = 100u;
    }
    uint16_t active = active_count();
    uint8_t feed_active = feed_active_count();
    uint8_t continuity_feed_limit =
        (s_flow_stage >= 2 || s_release_lookahead > 2.0f)
        ? HG_UPPER_HARD_STALL_ACTIVE_LIMIT
        : HG_UPPER_FEED_ACTIVE_LIMIT;
    uint8_t wake_budget = HG_CONTINUITY_WAKE_BUDGET;
    uint8_t light_relax_budget = 12;
    bool repaired = relax_upper_stable_pile();
    s_upper_surface_active_count = 0;
    s_upper_subsurface_active_count = 0;
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (!is_active(particle) ||
            particle->state != HG_PARTICLE_UPPER_ACTIVE ||
            !in_upper_supply_zone(particle)) {
            continue;
        }
        uint8_t above = particle_count_above(i);
        if (above == 0 && s_upper_surface_active_count < 255) {
            ++s_upper_surface_active_count;
        } else if (above <= 2 &&
                   s_upper_subsurface_active_count < 255) {
            ++s_upper_subsurface_active_count;
        }
    }
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (particle->state != HG_PARTICLE_UPPER_SLEEPING ||
            particle->y < 20.0f ||
            particle->y >= (float)HG_NECK_TOP_Y) {
            continue;
        }
        uint8_t supports = support_count_below(i);
        uint8_t above = particle_count_above(i);
        uint8_t neighbors = nearby_particle_count(i);
        int wall_y = boundary_row(particle->y);
        float clearance = particle_wall_clearance(particle);
        float wall_gap = fminf(
            particle->x -
                (effective_left_boundary(wall_y) + clearance),
            (effective_right_boundary(wall_y) - clearance) -
                particle->x);
        bool wall_supported = wall_gap <= HG_BOUNDARY_SLEEP_GAP;
        bool unsupported = supports == 0 && !wall_supported;
        bool weak_bridge = supports == 1 && above <= 2;
        bool void_edge = neighbors < 3;
        bool surface_or_subsurface = above <= 2;
        bool new_fault_event = particle->stable_ticks != 0;
        if (!unsupported && !weak_bridge && !void_edge &&
            !(surface_or_subsurface &&
              ((i + s_step_counter) % 7u == 0u))) {
            continue;
        }
        bool can_activate =
            in_upper_supply_zone(particle) && wake_budget > 0 &&
            active < active_limit &&
            feed_active < continuity_feed_limit;
        if (can_activate) {
            particle->state = HG_PARTICLE_UPPER_ACTIVE;
            particle->stable_ticks = 0;
            particle->vy += unsupported ? 0.28f : 0.10f;
            if (particle->x < HG_CENTER_X) {
                particle->vx += 0.04f;
            } else {
                particle->vx -= 0.04f;
            }
            --wake_budget;
            ++active;
            ++feed_active;
            repaired = true;
        } else if ((unsupported || void_edge) &&
                   light_relax_budget > 0 && particle->y < 54.4f) {
            float compact_step = s_flow_stage >= 2 ? 0.18f : 0.08f;
            float center_step =
                particle->x < HG_CENTER_X ? compact_step : -compact_step;
            float candidate_x = particle->x + center_step;
            float candidate_y = particle->y + 0.12f;
            if (!position_overlaps_neighbor(i, candidate_x, candidate_y)) {
                particle->x = candidate_x;
                particle->y = candidate_y;
                particle->stable_ticks = 0;
                constrain_to_glass(particle);
                --light_relax_budget;
                repaired = true;
            }
        }
        if ((unsupported || void_edge) && new_fault_event) {
            if (s_upper_continuity_fault_count < UINT16_MAX) {
                ++s_upper_continuity_fault_count;
            }
        }
        if (weak_bridge && new_fault_event &&
            s_bridge_break_count < UINT16_MAX) {
            ++s_bridge_break_count;
        }
        if (unsupported || weak_bridge || void_edge) {
            particle->stable_ticks = 0;
        }
    }

    if (repaired || new_faults != 0 || force) {
        rebuild_grid();
        solve_upper_funnel_collisions();
    }
}

static void relax_boundary_particles(void)
{
    /*
     * Inspect only a rotating, fixed-size subset. This covers all 300 grains
     * over several 20 Hz steps without turning sleeping sand into a full
     * per-frame simulation.
     */
    rebuild_grid();
    uint16_t available =
        active_count() < s_max_active ? s_max_active - active_count() : 0;
    for (uint8_t checked = 0;
         checked < HG_BOUNDARY_RELAX_BUDGET && s_particle_count > 0;
         ++checked) {
        uint16_t index = s_boundary_relax_cursor++ % s_particle_count;
        hg_particle_t *particle = &s_particles[index];
        int y = boundary_row(particle->y);
        float clearance = particle_wall_clearance(particle);
        float left_gap =
            particle->x - (effective_left_boundary(y) + clearance);
        float right_gap =
            (effective_right_boundary(y) - clearance) - particle->x;
        bool use_left = left_gap <= right_gap;
        float wall_gap = use_left ? left_gap : right_gap;
        if (wall_gap <= HG_BOUNDARY_SLEEP_GAP ||
            wall_gap >= HG_BOUNDARY_NEAR_GAP) {
            continue;
        }

        float normal_x =
            use_left ? s_left_normal_x[y] : s_right_normal_x[y];
        float normal_y =
            use_left ? s_left_normal_y[y] : s_right_normal_y[y];
        float correction =
            fminf(HG_BOUNDARY_RELAX_STEP,
                  wall_gap - HG_BOUNDARY_SLEEP_GAP);
        float candidate_x = particle->x - normal_x * correction;
        float candidate_y = particle->y - normal_y * correction;
        if (candidate_y < particle->y) {
            candidate_y = particle->y;
        }
        bool moved =
            !position_overlaps_neighbor(index, candidate_x, candidate_y);
        if (moved) {
            particle->x = candidate_x;
            particle->y = candidate_y;
            constrain_to_glass(particle);
        }

        bool neck_particle = particle->y >= 45.0f && particle->y <= 70.0f;
        bool exposed_surface = !has_particle_above(index);
        if (!moved && is_sleeping(particle) && wall_gap > 1.2f &&
            (neck_particle || exposed_surface) && available > 0) {
            particle->state =
                particle->y < (float)HG_NECK_TOP_Y
                ? HG_PARTICLE_UPPER_ACTIVE
                : HG_PARTICLE_LOWER_ACTIVE;
            particle->stable_ticks = 0;
            --available;
        }
    }
}

static uint8_t wake_nearest_throat_particles(uint8_t budget)
{
    uint16_t current_active = active_count();
    uint16_t throat_active_limit = (uint16_t)s_max_active + 12u;
    if (throat_active_limit > 100u) {
        throat_active_limit = 100u;
    }
    uint8_t feed_active = feed_active_count();
    uint8_t feed_limit =
        (s_flow_stage >= 2 || s_release_lookahead > 2.0f)
        ? HG_UPPER_HARD_STALL_ACTIVE_LIMIT
        : HG_UPPER_FEED_ACTIVE_LIMIT;
    uint8_t woke = 0;
    while (woke < budget && current_active < throat_active_limit &&
           feed_active < feed_limit) {
        int best_index = -1;
        float best_score = -100000.0f;
        for (uint16_t i = 0; i < s_particle_count; ++i) {
            hg_particle_t *particle = &s_particles[i];
            if (particle->state != HG_PARTICLE_UPPER_SLEEPING) {
                continue;
            }
            bool throat_candidate = in_throat_zone(particle);
            bool funnel_surface =
                in_upper_supply_zone(particle) &&
                !has_support_below(i);
            if (!throat_candidate && !funnel_surface) {
                continue;
            }
            float score =
                particle->y * 4.0f -
                fabsf(particle->x - HG_CENTER_X) * 2.5f;
            if (!has_support_below(i)) {
                score += 24.0f;
            }
            if (score > best_score) {
                best_score = score;
                best_index = (int)i;
            }
        }
        if (best_index < 0) {
            break;
        }
        hg_particle_t *particle = &s_particles[best_index];
        particle->state = HG_PARTICLE_UPPER_ACTIVE;
        particle->stable_ticks = 0;
        particle->vy += 0.35f;
        ++woke;
        ++current_active;
        ++feed_active;
    }
    return woke;
}

static void maintain_throat_activity(uint16_t upper_count, bool allow_flow)
{
    if (!allow_flow || upper_count <= HG_FLOW_MIN_UPPER) {
        return;
    }
    uint8_t throat_active = 0;
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (is_active(particle) && in_throat_zone(particle)) {
            ++throat_active;
        }
    }
    if (throat_active < HG_THROAT_MIN_ACTIVE) {
        wake_nearest_throat_particles(HG_THROAT_MIN_ACTIVE - throat_active);
        rebuild_grid();
    }
}

static void relax_throat_arch(void)
{
    rebuild_grid();
    uint8_t moved = 0;
    for (uint16_t i = 0; i < s_particle_count && moved < 8; ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (!is_active(particle) || !in_throat_core(particle) ||
            particle->y > (float)HG_NECK_BOTTOM_Y + 1.0f) {
            continue;
        }
        float side = (i & 1u) == 0u ? -0.08f : 0.08f;
        float candidate_x = particle->x + side;
        float candidate_y = particle->y + 0.30f;
        if (!position_overlaps_neighbor(i, candidate_x, candidate_y)) {
            particle->x = candidate_x;
            particle->y = candidate_y;
            particle->vy += 0.18f;
            constrain_to_glass(particle);
            ++moved;
            if (s_bridge_break_count < UINT16_MAX) {
                ++s_bridge_break_count;
            }
        }
    }
    rebuild_grid();
}

static void update_flow_watchdog(uint16_t target, uint16_t transferred,
                                 uint16_t upper_count, bool allow_flow)
{
    if (s_recovery_passes >= 3 && s_throat_expansion > 0.0f) {
        s_throat_expansion =
            fmaxf(0.0f, s_throat_expansion - 0.01f);
        if (s_throat_expansion == 0.0f) {
            s_recovery_passes = 0;
        }
    }
    bool flow_due = target > transferred;
    if (!allow_flow || upper_count <= HG_FLOW_MIN_UPPER || !flow_due) {
        s_flow_demand_active = false;
        s_flow_stage = 0;
        if (s_throat_expansion > 0.0f &&
            (s_recovery_passes >= 3 || !allow_flow)) {
            s_throat_expansion =
                fmaxf(0.0f, s_throat_expansion - 0.01f);
        }
        return;
    }
    if (!s_flow_demand_active) {
        s_flow_demand_active = true;
        s_flow_demand_start_step = s_step_counter;
        s_flow_stage = 0;
    }

    maintain_throat_activity(upper_count, true);
    uint32_t reference_step =
        s_last_particle_pass_step > s_flow_demand_start_step
        ? s_last_particle_pass_step
        : s_flow_demand_start_step;
    uint32_t stalled_ticks = s_step_counter - reference_step;
    if (stalled_ticks < HG_FLOW_SUSPECT_TICKS) {
        return;
    }

    uint32_t since_action = s_step_counter - s_last_unblock_step;
    if (s_flow_stage == 0) {
        if (s_stall_detect_count < UINT16_MAX) {
            ++s_stall_detect_count;
        }
        wake_nearest_throat_particles(HG_FLOW_WAKE_BUDGET);
        upper_layer_continuity_check(true);
        rebuild_grid();
        solve_throat_collisions();
        s_flow_stage = 1;
    } else if (since_action >= 4 && s_flow_stage == 1) {
        s_throat_low_friction_ticks = HG_FLOW_LOW_FRICTION_TICKS;
        upper_layer_continuity_check(true);
        s_flow_stage = 2;
    } else if (stalled_ticks >= HG_FLOW_HARD_STALL_TICKS &&
               since_action >= 4 && s_flow_stage == 2) {
        relax_throat_arch();
        rebuild_grid();
        solve_throat_collisions();
        s_flow_stage = 3;
    } else if (since_action >= 4 && s_flow_stage == 3) {
        s_recovery_passes = 0;
        s_flow_stage = 4;
    } else if (s_flow_stage == 4) {
        s_throat_expansion = fminf(
            HG_THROAT_MAX_EXPANSION, s_throat_expansion + 0.01f);
        if (since_action >= 10) {
            wake_nearest_throat_particles(4);
            relax_throat_arch();
        } else {
            return;
        }
    } else {
        return;
    }
    s_last_unblock_step = s_step_counter;
    ++s_unblock_count;
}

static void enforce_throat_boundaries(void)
{
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        if (in_throat_zone(&s_particles[i])) {
            constrain_to_glass(&s_particles[i]);
        }
    }
    rebuild_grid();
}

void hg_physics_init(void)
{
    init_geometry();
    hg_physics_reset();
}

void hg_physics_reset(void)
{
    s_particle_count = s_requested_particle_count;
    memset(s_particles, 0, sizeof(s_particles));
    uint16_t index = 0;
    int row = 0;
    for (float y = 53.0f;
         y >= 11.0f && index < s_particle_count;
         y -= HG_STABLE_ROW_STEP, ++row) {
        int boundary_y = (int)(y + 0.5f);
        float spacing = 2.90f;
        float start =
            s_left_boundary[boundary_y] + 1.5f +
            (row & 1 ? spacing * 0.5f : 0.0f);
        float right = s_right_boundary[boundary_y] - 1.5f;
        int row_slots = (int)((right - start) / spacing) + 1;
        for (int rank = 0;
             rank < row_slots && index < s_particle_count;
             ++rank) {
            float x =
                start +
                (float)centered_row_slot(rank, row_slots) * spacing;
            hg_particle_t *particle = &s_particles[index];
            particle->x = x;
            particle->y = y;
            particle->size = index % 10u == 0u ? 3u : 2u;
            particle->state = HG_PARTICLE_UPPER_SLEEPING;
            particle->stable_ticks = HG_SLEEP_TICKS;
            particle->color_phase = 0;
            constrain_to_glass(particle);
            ++index;
        }
    }
    while (index < s_particle_count) {
        hg_particle_t *particle = &s_particles[index];
        particle->x = 25.0f + (float)((index * 17u) % 64u);
        particle->y = 12.0f + (float)((index * 11u) % 13u);
        particle->size = index % 10u == 0u ? 3u : 2u;
        particle->state = HG_PARTICLE_UPPER_SLEEPING;
        particle->stable_ticks = HG_SLEEP_TICKS;
        constrain_to_glass(particle);
        ++index;
    }
    s_last_transferred = 0;
    s_step_counter = 0;
    s_last_gravity_x = 0.0f;
    s_all_transferred = false;
    s_allow_upper_reactivation = false;
    s_boundary_relax_cursor = 0;
    s_last_particle_pass_step = 0;
    s_last_unblock_step = 0;
    s_unblock_count = 0;
    s_throat_low_friction_ticks = 0;
    s_upper_collision_wake_budget = 0;
    s_recovery_passes = 0;
    s_flow_stage = 0;
    s_throat_expansion = 0.0f;
    s_flow_demand_start_step = 0;
    s_flow_demand_active = false;
    s_last_progress = 0.0f;
    s_release_lookahead = 0.0f;
    s_flow_pass_quota = 0;
    s_stall_detect_count = 0;
    s_upper_continuity_fault_count = 0;
    s_detected_void_count = 0;
    s_bridge_break_count = 0;
    s_void_band_mask = 0;
    s_upper_surface_active_count = 0;
    s_upper_subsurface_active_count = 0;
    s_flow_release_due = false;
    s_compaction_gravity_x = 0.0f;
    s_compaction_gravity_y = 1.0f;
    s_last_compaction_remap_upper = s_particle_count;
    memset(s_upper_target_assignment, 0xff,
           sizeof(s_upper_target_assignment));
    rebuild_grid();
}

void hg_physics_set_limits(uint8_t max_active, uint8_t constraint_iterations)
{
    if (max_active < 40) {
        max_active = 40;
    } else if (max_active > 100) {
        max_active = 100;
    }
    if (constraint_iterations < 2) {
        constraint_iterations = 2;
    } else if (constraint_iterations > 3) {
        constraint_iterations = 3;
    }
    s_max_active = max_active;
    s_constraint_iterations = constraint_iterations;
}

void hg_physics_request_particle_count(uint16_t particle_count)
{
    if (particle_count < 220) {
        particle_count = 220;
    } else if (particle_count > HG_PARTICLE_CAPACITY) {
        particle_count = HG_PARTICLE_CAPACITY;
    }
    s_requested_particle_count = particle_count;
}

void hg_physics_wake_surface(void)
{
    rebuild_grid();
    uint16_t available =
        active_count() < s_max_active ? s_max_active - active_count() : 0;
    for (uint16_t i = 0; i < s_particle_count && available > 0; ++i) {
        hg_particle_t *particle = &s_particles[i];
        if (is_sleeping(particle) &&
            (!has_particle_above(i) || !has_support_below(i))) {
            wake_particle(
                i, particle->state == HG_PARTICLE_UPPER_SLEEPING
                       ? HG_PARTICLE_UPPER_ACTIVE
                       : HG_PARTICLE_LOWER_ACTIVE);
            --available;
        }
    }
}

void hg_physics_step(float gravity_x, float gravity_y, float progress,
                     bool allow_flow)
{
    ++s_step_counter;
    gravity_x = clampf(gravity_x, -0.82f, 0.82f);
    gravity_y = clampf(gravity_y, 0.25f, 1.0f);
    float gravity_length =
        sqrtf(gravity_x * gravity_x + gravity_y * gravity_y);
    if (gravity_length > 0.1f) {
        gravity_x /= gravity_length;
        gravity_y /= gravity_length;
    }
    s_compaction_gravity_x =
        s_compaction_gravity_x * 0.88f + gravity_x * 0.12f;
    s_compaction_gravity_y =
        s_compaction_gravity_y * 0.88f + gravity_y * 0.12f;

    uint16_t transferred = transferred_count();
    float progress_delta = fmaxf(0.0f, progress - s_last_progress);
    /*
     * Keep roughly one second of material in the physical pipeline. Since
     * this derives from progress per fixed step, it scales automatically for
     * every selected duration. Ramp during the first two seconds to avoid a
     * visible startup burst in the lower chamber.
     */
    float elapsed_steps = progress_delta > 0.0000001f
        ? progress / progress_delta
        : 40.0f;
    float pipeline_ramp = clampf(elapsed_steps / 40.0f, 0.0f, 1.0f);
    float pipeline_particles =
        progress_delta * (float)s_particle_count * 20.0f * pipeline_ramp;
    s_release_lookahead =
        clampf(pipeline_particles, 0.0f, 18.0f);
    s_last_progress = progress;
    uint16_t flow_target = (uint16_t)clampf(
        floorf(progress * (float)s_particle_count +
               s_release_lookahead + 0.001f),
        0.0f, (float)s_particle_count);
    s_all_transferred = transferred == s_particle_count;
    s_allow_upper_reactivation = transferred <= flow_target;
    uint8_t feed_active = feed_active_count();
    s_upper_collision_wake_budget =
        flow_target > transferred && feed_active < 4u ? 1u : 0u;
    s_flow_release_due = allow_flow && flow_target > transferred;
    uint16_t pass_deficit =
        s_flow_release_due ? flow_target - transferred : 0u;
    s_flow_pass_quota = pass_deficit > 0u ? 1u : 0u;
    rebuild_grid();
    wake_for_target(progress, allow_flow);
    wake_funnel_and_slopes(gravity_x, progress);
    if (fabsf(gravity_x - s_last_gravity_x) > 0.18f) {
        hg_physics_wake_surface();
        s_last_gravity_x = gravity_x;
    }

    integrate_particles(gravity_x, gravity_y);
    for (uint8_t iteration = 0; iteration < s_constraint_iterations;
         ++iteration) {
        rebuild_grid();
        solve_particle_collisions();
    }
    rebuild_grid();
    solve_throat_collisions();
    enforce_flow_gate();
    update_regions_and_sleep();
    upper_layer_continuity_check(false);
    relax_boundary_particles();

    uint16_t updated_transferred = transferred_count();
    if (updated_transferred != s_last_transferred) {
        s_last_transferred = updated_transferred;
    }
    if (s_throat_low_friction_ticks > 0) {
        --s_throat_low_friction_ticks;
    }
    uint16_t upper_count = s_particle_count - updated_transferred;
    update_flow_watchdog(
        flow_target, updated_transferred, upper_count, allow_flow);
    enforce_throat_boundaries();
}

const hg_particle_t *hg_physics_particles(void)
{
    return s_particles;
}

uint16_t hg_physics_particle_count(void)
{
    return s_particle_count;
}

hg_physics_stats_t hg_physics_stats(void)
{
    hg_physics_stats_t stats = {
        .total = s_particle_count,
        .settled = true,
    };
    for (uint16_t i = 0; i < s_particle_count; ++i) {
        const hg_particle_t *particle = &s_particles[i];
        if (is_active(particle)) {
            ++stats.active;
            stats.settled = false;
        } else if (is_sleeping(particle)) {
            ++stats.sleeping;
        }
        if (in_throat_zone(particle)) {
            ++stats.throat_particles;
            if (is_active(particle)) {
                ++stats.throat_active;
            }
        }
        if (particle->state == HG_PARTICLE_UPPER_SLEEPING ||
            particle->state == HG_PARTICLE_UPPER_ACTIVE) {
            ++stats.upper;
        } else if (particle->state == HG_PARTICLE_FALLING) {
            ++stats.falling;
        } else if (particle->state == HG_PARTICLE_LOWER_ACTIVE ||
                   particle->state == HG_PARTICLE_LOWER_SLEEPING) {
            ++stats.lower;
        }
    }
    stats.transferred = stats.falling + stats.lower;
    uint32_t reference_step =
        s_last_particle_pass_step > s_flow_demand_start_step
        ? s_last_particle_pass_step
        : s_flow_demand_start_step;
    uint32_t stall_ticks =
        s_flow_demand_active ? s_step_counter - reference_step : 0;
    uint32_t stall_ms = stall_ticks * (1000u / HG_PHYSICS_HZ);
    stats.flow_stall_ms =
        stall_ms > UINT16_MAX ? UINT16_MAX : (uint16_t)stall_ms;
    stats.stall_detect_count = s_stall_detect_count;
    stats.unblock_count = s_unblock_count;
    stats.upper_continuity_fault_count =
        s_upper_continuity_fault_count;
    stats.detected_void_count = s_detected_void_count;
    stats.bridge_break_count = s_bridge_break_count;
    stats.upper_surface_active = s_upper_surface_active_count;
    stats.upper_subsurface_active =
        s_upper_subsurface_active_count;
    float timer_expansion =
        clampf((s_release_lookahead - 2.0f) * 0.02f, 0.0f,
               HG_THROAT_MAX_EXPANSION);
    stats.effective_throat_width =
        HG_THROAT_BASE_WIDTH *
        (1.0f + fmaxf(s_throat_expansion, timer_expansion));
    return stats;
}
