import pygame
import sys
import numpy as np

# Pygame 초기화
pygame.init()

# 화면 설정
width, height = 100, 100
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("SPH Particle Simulation")
FPS = 60

# 색깔 정의
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)

# 시뮬레이션 매개변수
num_particles = 100
particle_radius = 5
smoothing_length = 20.0
rho_0 = 1.0
gas_constant = 1.0
dt = 0.1

# 입자 클래스 정의
class Particle:
    def __init__(self, x, y, vx, vy, mass):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.mass = mass

# 입자 초기화
particles = [Particle(
    x=np.random.uniform(particle_radius, width - particle_radius),
    y=np.random.uniform(height/2, height),
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
    EPSILON = 1e-9
    for i in range(len(particles)):
        particles[i].fx = 0.0
        particles[i].fy = 0.0
        for j in range(len(particles)):
            if i != j:
                dist_x = particles[i].x - particles[j].x
                dist_y = particles[i].y - particles[j].y
                dist = np.sqrt(dist_x**2 + dist_y**2)

                # Softening Function을 적용하여 0으로 나누는 상황 방지
                softening_term = EPSILON * smoothing_length

                pressure_grad_x = -particles[i].mass * (particles[i].pressure + particles[j].pressure) / (
                    2 * (particles[j].density + softening_term)) * poly6_kernel(dist_x, h)
                pressure_grad_y = -particles[i].mass * (particles[i].pressure + particles[j].pressure) / (
                    2 * (particles[j].density + softening_term)) * poly6_kernel(dist_y, h)
                particles[i].fx += pressure_grad_x
                particles[i].fy += pressure_grad_y

def compute_vicosity_force(particles, h):
    for i in range(len(particles)):
        for j in range(len(particles)):
            if i != j:
                vel_x = particles[i].vx - particles[j].vx
                vel_y = particles[i].vy - particles[j].vy
                dist_x = particles[i].x - particles[j].x
                dist_y = particles[i].y - particles[j].y

                

def compute_attraction_force(particles):
    for i in range(len(particles)):
        for j in range(i+1,len(particles)):
            dist_x = particles[i].x - particles[j].x
            dist_y = particles[i].y - particles[j].y
            dist = np.sqrt(dist_x**2 + dist_y**2)
            dir_x = dist_x / dist
            dir_y = dist_y / dist
            particles[i].fx = dir_x * particles[i].mass * particles[j].mass / dist**2
            particles[j].fx = dir_y * particles[i].mass * particles[j].mass / dist**2

def update_gravity(particles, dt):
    for i in range(len(particles)):
        particles[i].vy += 9.8 * dt

def update_particles(particles, dt):
    for i in range(len(particles)):
        particles[i].vx += particles[i].fx * dt / particles[i].mass
        particles[i].vy += particles[i].fy * dt / particles[i].mass
        particles[i].x += particles[i].vx * dt
        particles[i].y += particles[i].vy * dt

# 메인 루프
clock = pygame.time.Clock()
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    compute_density_and_pressure(particles, smoothing_length)
    compute_forces(particles, smoothing_length)
    compute_attraction_force(particles)
    update_gravity(particles, dt)
    update_particles(particles, dt)

    eplison = 0
    for particle in particles:
        if particle.x < -width + eplison:
            particle.x = -width + eplison
            particle.vx = -particle.vx
        if particle.x > width - eplison:
            particle.x = width - eplison
            particle.vx = -particle.vx
        if particle.y < -height + eplison:
            particle.y = -height - eplison
            particle.vy = -particle.vy
        if particle.y > height - eplison:
            particle.y = height - eplison
            particle.vy = -particle.vy

    # 화면 클리어
    screen.fill(WHITE)

    # 입자 그리기
    for particle in particles:
        pygame.draw.circle(screen, BLUE, (int(particle.x), int(particle.y)), particle_radius)

    # 화면 업데이트
    pygame.display.flip()
    clock.tick(FPS)  # FPS 설정
