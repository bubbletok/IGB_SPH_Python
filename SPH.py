import pygame
import numpy as np

# Pygame setup
pygame.init()

# Initial setup
width, height = 100, 100
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("SPH Particle Simulation")
FPS = 60

# Color setup
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)

# Particle setup
num_particles = 50
particle_radius = 5
smoothing_length = 20.0
rho_0 = 1.0
gas_constant = 1.0
dt = 0.1

# Particle class
class Particle:
    def __init__(self, x, y, vx, vy, mass):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.fx = 0.0
        self.fy = 0.0
        self.mass = mass

# Particle initialization
particles = [Particle(
    x=np.random.uniform(width/4, width/2),
    y=np.random.uniform(height/4, height/2),
    vx=0.0,
    vy=0.0,
    mass=1.0
) for _ in range(num_particles)]

def poly6_kernel(r, h):
    if 0 <= r <= h:
        return 315 / (64 * np.pi * h**9) * (h**2 - r**2)**3
    else:
        return 0.0

def compute_density_and_pressure(particles, h):
    for i in range(len(particles)):
        particles[i].density = 0.0
        for j in range(len(particles)):
            if i != j:
                dist = np.sqrt((particles[i].x - particles[j].x)**2 + (particles[i].y - particles[j].y)**2)
                particles[i].density += particles[j].mass * poly6_kernel(dist, h)
        particles[i].pressure = gas_constant * (particles[i].density - rho_0)

def compute_forces(particles, h):
    EPSILON = 1e-2
    for i in range(len(particles)):
        particles[i].fx = 0.0
        particles[i].fy = 0.0
        for j in range(len(particles)):
            if i != j:
                dist_x = particles[i].x - particles[j].x
                dist_y = particles[i].y - particles[j].y
                dist = np.sqrt(dist_x**2 + dist_y**2)

                # Softening Function to prevent division by zero
                softening_term = EPSILON * smoothing_length

                pressure_grad_x = -particles[i].mass * (particles[i].pressure + particles[j].pressure) / (
                    2 * (particles[j].density + softening_term)) * poly6_kernel(dist_x, h)
                pressure_grad_y = -particles[i].mass * (particles[i].pressure + particles[j].pressure) / (
                    2 * (particles[j].density + softening_term)) * poly6_kernel(dist_y, h)
        particles[i].fx += pressure_grad_x
        particles[i].fy += pressure_grad_y

def compute_vicosity_force(particles, h):
    EPSILON = 1e-2
    mu = dynamic_viscosity = 1.002
    for i in range(len(particles)):
        for j in range(len(particles)):
            if i != j:
                vel_x = particles[i].vx - particles[j].vx
                vel_y = particles[i].vy - particles[j].vy
                dist_x = particles[i].x - particles[j].x
                dist_y = particles[i].y - particles[j].y
                force_x = 0.0
                force_y = 0.0

                # Softening Function to prevent division by zero
                softening_term = EPSILON * smoothing_length

                force_x += particles[j].mass * (vel_x/(particles[j].density + softening_term)) * (poly6_kernel(dist_x, h)**2)
                force_y += particles[j].mass * (vel_y/(particles[j].density + softening_term)) * (poly6_kernel(dist_y, h)**2)

        particles[i].fx += mu * force_x
        particles[i].fy += mu * force_y

def check_collision(particles):
    for i in range(len(particles)):
        for j in range(len(particles)):
            if i != j:
                dist_x = particles[i].x - particles[j].x
                dist_y = particles[i].y - particles[j].y
                dist = np.sqrt(dist_x**2 + dist_y**2)
                if dist <= 2 * particle_radius:
                    # get direction
                    dir_x = dist_x / dist
                    dir_y = dist_y / dist
                    # move particles
                    particles[i].x = particles[j].x - dir_x
                    particles[i].y = particles[j].y - dir_y
                    particles[i].vx /=2
                    particles[i].vy /=2

# def compute_attraction_force(particles):
#     for i in range(len(particles)):
#         for j in range(i+1,len(particles)):
#             dist_x = particles[i].x - particles[j].x
#             dist_y = particles[i].y - particles[j].y
#             dist = np.sqrt(dist_x**2 + dist_y**2)
#             if dist!=0:
#                 dir_x = dist_x / dist
#                 dir_y = dist_y / dist
#                 particles[i].fx = dir_x * particles[i].mass * particles[j].mass / dist**2
#                 particles[j].fx = dir_y * particles[i].mass * particles[j].mass / dist**2

def update_gravity(particles, dt):
    for i in range(len(particles)):
        particles[i].vy += 9.8 * dt

def update_particles(particles, dt):
    for i in range(len(particles)):
        particles[i].vx += particles[i].fx * dt / particles[i].mass
        particles[i].vy += particles[i].fy * dt / particles[i].mass
        particles[i].x += particles[i].vx * dt
        particles[i].y += particles[i].vy * dt

def update_XSPH(particles, h):
    eplison = 1e-9
    EPSILON = 1e-9
    for i in range(len(particles)):
        for j in range(len(particles)):
            if i!=j:
                dist_x = particles[i].x - particles[j].x
                dist_y = particles[i].y - particles[j].y
                vel_x = particles[i].vx - particles[j].vx
                vel_y = particles[i].vy - particles[j].vy

                temp_correction_x = 0.0
                temp_correction_y = 0.0

                # Softening Function to prevent division by zero
                softening_term = EPSILON * smoothing_length

                temp_correction_x += (2*particles[j].mass * vel_x)/(particles[i].density+particles[j].density + softening_term)
                temp_correction_y += (2*particles[j].mass * vel_y)/(particles[i].density+particles[j].density + softening_term)
        temp_correction_x *= eplison * vel_x * poly6_kernel(dist_x, h)
        temp_correction_y *= eplison * vel_y * poly6_kernel(dist_y, h)

        particles[i].x += temp_correction_x
        particles[i].y += temp_correction_y


# Main loop
clock = pygame.time.Clock()
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
        if pygame.key.get_pressed()[pygame.K_SPACE]:
            print("Space bar pressed")
            for particle in particles:
                particle.vx = np.random.uniform(-5,5)
                particle.vy = np.random.uniform(-100,100)
        
    compute_density_and_pressure(particles, smoothing_length)
    compute_forces(particles, smoothing_length)
    compute_vicosity_force(particles, smoothing_length)
    # compute_attraction_force(particles)

    update_gravity(particles, dt)
    update_particles(particles, dt)

    update_XSPH(particles, smoothing_length)

    check_collision(particles)

    eplison = 0
    for particle in particles:
        if particle.x < 0:
            particle.x = eplison
            particle.vx = -particle.vx/2
        if particle.x > width - eplison:
            particle.x = width - eplison
            particle.vx = -particle.vx/2
        if particle.y < 0 + eplison:
            particle.y =  eplison
            particle.vy = -particle.vy/2
        if particle.y > height - eplison:
            particle.y = height - eplison
            particle.vy = -particle.vy/2

    # for particle in particles:
    #     if particle.x < -width or particle.x > width:
    #         particle.x = width/2
    #     if particle.y < -height or particle.y > height:
    #         particle.y = height/2

    screen.fill(WHITE)

    # Draw particles
    for particle in particles:
        pygame.draw.circle(screen, BLUE, ((particle.x), (particle.y)), particle_radius)

    pygame.display.flip()
    clock.tick(FPS)  # FPS
