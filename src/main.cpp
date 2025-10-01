#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include "Pendulum.hpp"
#include "constants.hpp"

#define PI 3.1415926535897932384626433
#define WORLD_SCALE 300.f

sf::Vector2f toScreen(sf::Vector2f windowCenter, sf::Vector2f pos)
{
    return sf::Vector2f(windowCenter.x + pos.x * WORLD_SCALE, windowCenter.y - pos.y * WORLD_SCALE);
}

int main()
{
    double m1 = 1.0, m2 = 1.0;
    double L1 = 0.5, L2 = 0.5;
    double theta1 = PI, theta2 = PI;
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

    sf::Clock clock;
    double accumulator = 0.0f;
    const double dt = 0.001f;

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

        sf::Vector2f pos1(x1, y1);
        sf::Vector2f pos2(x2, y2);

        window.clear(COLOR::DARK_GRAY);

        sf::CircleShape pivot(10.f);
        pivot.setPointCount(50);
        pivot.setFillColor(COLOR::GRAY);
        pivot.setOrigin(sf::Vector2f(10.f, 10.f));
        pivot.setPosition(origin);
        window.draw(pivot);

        sf::VertexArray rod1(sf::PrimitiveType::Lines, 2);
        rod1[0].position = origin;
        rod1[0].color = COLOR::GRAY;
        rod1[1].position = toScreen(origin, pos1);
        rod1[1].color = COLOR::GRAY;
        window.draw(rod1);

        sf::VertexArray rod2(sf::PrimitiveType::Lines, 2);
        rod2[0].position = toScreen(origin, pos1);
        rod2[0].color = COLOR::GRAY;
        rod2[1].position = toScreen(origin, pos2);
        rod2[1].color = COLOR::GRAY;
        window.draw(rod2);

        sf::CircleShape bob1(20.f);
        bob1.setPointCount(50);
        bob1.setFillColor(COLOR::BLUE);
        bob1.setOrigin(sf::Vector2f(20.f, 20.f));
        bob1.setPosition(toScreen(origin, pos1));
        window.draw(bob1);

        sf::CircleShape bob2(20.f);
        bob2.setPointCount(50);
        bob2.setFillColor(COLOR::RED);
        bob2.setOrigin(sf::Vector2f(20.f, 20.f));
        bob2.setPosition(toScreen(origin, pos2));
        window.draw(bob2);

        window.display();
    }
}
