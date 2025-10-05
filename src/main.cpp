#include <SFML/Graphics.hpp>
#include <cmath>
#include <deque>
#include <iostream>
#include "Pendulum.hpp"
#include "Slider.hpp"
#include "constants.hpp"

#define PI 3.1415926535897932384626433
#define WORLD_SCALE 250.f

sf::Vector2f toScreen(sf::Vector2f windowCenter, sf::Vector2f pos)
{
    return sf::Vector2f(windowCenter.x + pos.x * WORLD_SCALE, windowCenter.y - pos.y * WORLD_SCALE);
}

int main()
{
    double m1 = 1.0, m2 = 1.0;
    double L1 = 0.5, L2 = 0.5;
    double theta1 = PI / 2, theta2 = PI / 2;
    
    DoublePendulum pendulum(m1, m2, L1, L2, theta1, theta2);

    sf::ContextSettings settings;
    settings.antiAliasingLevel = 8;

    auto window = sf::RenderWindow(
        sf::VideoMode({1920u, 1080u}), 
        "Double Pendulum Animation", 
        sf::Style::Default,
        sf::State::Windowed,
        settings
    );
    window.setFramerateLimit(144);

    sf::Vector2u size = window.getSize();
    double width = static_cast<double>(size.x);
    double height = static_cast<double>(size.y);
    sf::Vector2f origin(width / 2.0f, height / 2.0f - 300.f);
    const float rod_thickness = 6.f;

    sf::Clock clock;
    double accumulator = 0.0f;
    const double dt = 0.001f;

    sf::CircleShape pivot(10.f);
    pivot.setPointCount(50);
    pivot.setFillColor(COLOR::FAINT_WHITE);
    pivot.setOrigin({10.f, 10.f});
    pivot.setPosition(origin);

    sf::RectangleShape rod1;
    rod1.setFillColor(COLOR::FAINT_WHITE);
    rod1.setSize({0, rod_thickness});
    rod1.setOrigin({0, rod_thickness / 2.f});

    sf::RectangleShape rod2;
    rod2.setFillColor(COLOR::FAINT_WHITE);
    rod2.setSize({0, rod_thickness});
    rod2.setOrigin({0, rod_thickness / 2.f});

    sf::CircleShape bob1(20.f);
    bob1.setPointCount(50);
    bob1.setFillColor(COLOR::BLUE);
    bob1.setOrigin({20.f, 20.f});

    sf::CircleShape bob2(20.f);
    bob2.setPointCount(50);
    bob2.setFillColor(COLOR::RED);
    bob2.setOrigin({20.f, 20.f});

    std::deque<sf::Vertex> trail;
    const size_t maxTrail = 500;

    while (window.isOpen())
    {
        while (std::optional<sf::Event> event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
        }

        double elapsed = clock.restart().asSeconds();
        accumulator += elapsed;

        while (accumulator >= dt)
        {
            pendulum.step(dt);
            accumulator -= dt;
        }
        
        auto [x1, y1] = pendulum.getBob1Position();
        auto [x2, y2] = pendulum.getBob2Position();

        sf::Vector2f screenPos1 = toScreen(origin, sf::Vector2f(x1, y1));
        sf::Vector2f screenPos2 = toScreen(origin, sf::Vector2f(x2, y2));

        sf::Vector2f d1 = screenPos1 - origin;
        float len1 = std::sqrt(d1.x * d1.x + d1.y * d1.y);
        float angle1 = std::atan2(d1.y, d1.x);

        rod1.setSize({len1, rod_thickness});
        rod1.setPosition(origin);
        rod1.setRotation(sf::radians(angle1));

        sf::Vector2f d2 = screenPos2 - screenPos1;
        float len2 = std::sqrt(d2.x * d2.x + d2.y * d2.y);
        float angle2 = std::atan2(d2.y, d2.x);

        rod2.setSize({len2, rod_thickness});
        rod2.setPosition(screenPos1);
        rod2.setRotation(sf::radians(angle2));

        bob1.setPosition(screenPos1);
        bob2.setPosition(screenPos2);

        trail.push_back({screenPos2, COLOR::GRAY});
        if (trail.size() > maxTrail)
            trail.pop_front();

        window.clear(COLOR::DARK_GRAY);

        if (!trail.empty())
        {
            sf::VertexArray curve(sf::PrimitiveType::LineStrip, trail.size());
            for (size_t i = 0; i < trail.size(); ++i)
            {
                curve[i] = trail[i];
            }
            window.draw(curve);
        }

        window.draw(rod1);
        window.draw(rod2);
        window.draw(bob1);
        window.draw(bob2);
        window.draw(pivot);

        window.display();
    }
}
