#include <Eigen/Dense>
#include <SDL2/SDL.h>
#include <algorithm>
#include <chrono>
#include <cxxopts.hpp>
#include <format>
#include <iostream>
#include <numeric>
#include <print>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

constexpr auto G = 6.67e-11;
constexpr auto CenterMass = 1.98847e30;// kg

double EarthVelocity(double squared_r) { return (CenterMass * G) / squared_r; }

struct Earth {
    Eigen::Vector2d position;
    Eigen::Vector2d speed;
};

Earth GetInitialEarth()
{
    return {
        .position = {1.496e11, 0.0},// m
        .speed = {-20e3, 20e3},     // m/s
        //.speed = {0.0, 29e3},    // m/s 29e3
    };
}

//Earth MoveBad(Earth earth, double seconds)
//{
//    const double velocity = EarthVelocity(earth.position.squaredNorm());
//    const auto norm = earth.position.normalized();
//    const auto old_rad_speed = norm.dot(earth.speed);
//    const auto new_rad_speed = old_rad_speed - norm.norm() * velocity * seconds;
//    const auto tan_norm = Eigen::Vector2d{-norm[1], norm[0]};
//    const auto tan_speed = tan_norm.dot(earth.speed);
//    earth.position +=
//        norm * (old_rad_speed * seconds + velocity * seconds * seconds / 2.0) + tan_norm * (tan_speed * seconds);
//    earth.speed = norm * new_rad_speed + tan_norm * tan_speed;
//    return earth;
//}

//void Move(Earth &earth, double seconds)
//{
//    Eigen::Vector2d r = earth.position;
//    double r_norm = r.norm();
//    Eigen::Vector2d gravitational_acceleration = -G * CenterMass / (r_norm * r_norm * r_norm) * r;
//
//    earth.speed += gravitational_acceleration * seconds;
//    earth.position += earth.speed * seconds;
//
//
//}
void Move(Earth &earth, double seconds)
{
    Eigen::Vector2d r = earth.position;
    double r_norm = r.norm();
    Eigen::Vector2d gravitational_acceleration = -G * CenterMass / (r_norm * r_norm * r_norm) * r;

    Eigen::Vector2d new_position =
        earth.position + earth.speed * seconds + 0.5 * gravitational_acceleration * seconds * seconds;

    Eigen::Vector2d new_r = new_position;
    double new_r_norm = new_r.norm();
    Eigen::Vector2d new_gravitational_acceleration = -G * CenterMass / (new_r_norm * new_r_norm * new_r_norm) * new_r;

    Eigen::Vector2d avg_acceleration = 0.5 * (gravitational_acceleration + new_gravitational_acceleration);
    earth.speed += avg_acceleration * seconds;
    earth.position = new_position;
}

template<> struct std::default_delete<SDL_Window> {
    void operator()(SDL_Window *window) { SDL_DestroyWindow(window); }
};

template<> struct std::default_delete<SDL_Renderer> {
    void operator()(SDL_Renderer *renderer) { SDL_DestroyRenderer(renderer); }
};

class Surface {
public:
    Surface()
        : window_(SDL_CreateWindow("SDL Line Drawing", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 0, 0,
                                   SDL_WINDOW_FULLSCREEN_DESKTOP | SDL_WINDOW_ALLOW_HIGHDPI))
    {
        if (!window_)
            throw std::runtime_error(SDL_GetError());
        renderer_.reset(SDL_CreateRenderer(window_.get(), -1, SDL_RENDERER_ACCELERATED));
        if (!renderer_)
            throw std::runtime_error(SDL_GetError());
    }
    void Clear(SDL_Color background)
    {
        SetColor(background);
        SDL_RenderClear(renderer_.get());
        SDL_SetRenderDrawBlendMode(renderer_.get(), SDL_BLENDMODE_NONE);
    }
    void Present() { SDL_RenderPresent(renderer_.get()); }
    void DrawPolyline(const std::vector<SDL_Point> &polyline, SDL_Color color)
    {
        SetColor(color);
        SDL_RenderDrawLines(renderer_.get(), polyline.data(), static_cast<int>(polyline.size()));
    }
    void DrawLine(SDL_Point from, SDL_Point to, SDL_Color color)
    {
        SetColor(color);
        SDL_RenderDrawLine(renderer_.get(), from.x, from.y, to.x, to.y);
    }
    void FillRect(const SDL_Rect rect, SDL_Color color)
    {
        SetColor(color);
        SDL_RenderFillRect(renderer_.get(), &rect);
    }
    SDL_Point Size()
    {
        SDL_Point result;
        SDL_GetRendererOutputSize(renderer_.get(), &result.x, &result.y);
        return result;
    }
    SDL_Point FromWindow(SDL_Point window_point)
    {
        float x, y;
        SDL_RenderWindowToLogical(renderer_.get(), window_point.x, window_point.y, &x, &y);
        return SDL_Point{
            .x = static_cast<int>(std::round(x)),
            .y = static_cast<int>(std::round(y)),
        };
    }

private:
    std::unique_ptr<SDL_Window> window_;
    std::unique_ptr<SDL_Renderer> renderer_;

    void SetColor(SDL_Color color) { SDL_SetRenderDrawColor(renderer_.get(), color.r, color.g, color.b, color.a); }
};

double MaxRadius(const std::vector<Eigen::Vector2d> &movement)
{
    auto max_xy = std::transform_reduce(
        movement.begin(), movement.end(), Eigen::Array2d{DBL_MIN, DBL_MIN},
        [](const Eigen::Array2d &a, const Eigen::Array2d &b) { return a.max(b); },
        [](const Eigen::Vector2d &v) { return v.array().abs(); });
    return std::max(max_xy[0], max_xy[1]);
}
void DrawAxis(Surface &surface, const SDL_Point &center, SDL_Color color)
{
    const auto surface_size = surface.Size();
    surface.DrawLine(SDL_Point(center.x, 0), SDL_Point(center.x, surface_size.y), color);
    surface.DrawLine(SDL_Point(0, center.y), SDL_Point(surface_size.x, center.y), color);
}

void Draw(Surface &surface, const std::vector<Eigen::Vector2d> &movement, const Eigen::Vector2d &speed)
{
    const auto max_movement_radius = MaxRadius(movement);
    const auto surface_size = surface.Size();
    const auto min_surface_radius = std::min(surface_size.x, surface_size.y) / 2.0 * 0.9;
    const auto center = SDL_Point{.x = surface_size.x / 2, .y = surface_size.y / 2};
    const auto scale = min_surface_radius / max_movement_radius;
    std::vector<SDL_Point> orbit(movement.size());
    std::transform(movement.begin(), movement.end(), orbit.begin(), [scale, center](const Eigen::Vector2d &pos) {
        return SDL_Point{
            .x = static_cast<int>(std::round(pos[0] * scale)) + center.x,
            .y = static_cast<int>(std::round(pos[1] * scale)) + center.y,
        };
    });
    const auto background_color = SDL_Color(0, 0, 0, 255);
    const auto axis_color = SDL_Color(120, 120, 120, 255);
    const auto orbit_color = SDL_Color(150, 150, 255, 255);
    const auto earth_color = SDL_Color(255, 255, 255, 255);
    surface.Clear(background_color);
    DrawAxis(surface, center, axis_color);
    surface.DrawPolyline(orbit, orbit_color);

    const auto &pos = orbit.back();
    surface.FillRect(SDL_Rect(pos.x - 5, pos.y - 5, 11, 11), earth_color);
    //const auto norm_speed = speed.normalized();
    const auto speed_scale = scale * 5e5;
    surface.DrawLine(pos,
                     SDL_Point{
                         .x = static_cast<int>(std::round(speed[0] * speed_scale)) + pos.x,
                         .y = static_cast<int>(std::round(speed[1] * speed_scale)) + pos.y,
                     },
                     earth_color);

    surface.Present();
}

void run(Surface &surface, Earth earth)
{
    const auto initial_distance = earth.position.squaredNorm();
    std::vector<Eigen::Vector2d> movement(1, earth.position);
    SDL_Event event;
    for (;;) {
        Draw(surface, movement, earth.speed);
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
            case SDL_QUIT:
                return;
            }
        }
        const auto time_coef = earth.position.squaredNorm() / initial_distance;
        //const auto time_coef = 1.0;
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(50 * time_coef));
        //std::this_thread::sleep_for(10ms);
        //        auto bad = MoveBad(earth, 60.0 * 60.0 * 24.0);
        //        std::print("Bad  Earth: speed=({:e}, {:e}), position=({:e}, {:e})\n", bad.speed[0], bad.speed[1], bad.position[0],
        //                   bad.position[1]);
        Move(earth, 60.0 * 60.0 * 24.0 * time_coef);
        //        std::print("Good Earth: speed=({:e}, {:e}), position=({:e}, {:e})\n", earth.speed[0], earth.speed[1], earth.position[0],
        //                   earth.position[1]);
        movement.push_back(earth.position);
    }
}

int main(int argc, char *args[])
{
    cxxopts::Options options("fly_simulator", "The Earth fly simulator");
    options.add_options()
        ("h,help", "show help")
        ("x,x_speed", "X axis speed in m/s", cxxopts::value<double>()->default_value("0"))
        ("y,y_speed", "Y axis speed in m/s", cxxopts::value<double>()->default_value("29e3"))
        ;
    auto parsed = options.parse(argc, args);
    if (parsed.count("help")){
        std::print("{}\n", options.help());
        return 0;
    }

    auto earth = GetInitialEarth();
    earth.speed[0] = parsed["x_speed"].as<double>();
    earth.speed[1] = parsed["y_speed"].as<double>();

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return 1;
    }
    try {
        Surface surface;
        run(surface, earth);
    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }
    SDL_Quit();

    return 0;
}
