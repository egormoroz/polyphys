#define _USE_MATH_DEFINES
#include <SFML/Graphics.hpp>
#include "body.hpp"
#include "manifold.hpp"
#include <random>
#include "world.hpp"

sf::Vector2f to_sf(const Vec2 &v) {
    return sf::Vector2f(v.x, v.y);
}

Body* static_rect(Vec2 top_left, Vec2 size) {
    Vec2 half = size / 2;
    Body *b = new Body();

    b->shape.num_vertices = 4;
    auto &vs = b->shape.vertices;
    vs[0] = Vec2(half.x, -half.y);
    vs[1] = Vec2(-half.x, -half.y);
    vs[2] = Vec2(-half.x, half.y);
    vs[3] = Vec2(half.x, half.y);

    b->pos = top_left + half;

    b->angle = 0;
    b->mass = 0.f;
    b->inv_mass = 0.f;
    b->restitution = 1.f;
    b->inv_inert = 0.f;
    b->intert = 0.f;

    return b;
}

Body* new_regular(Vec2 pos, int n, float size) {
    assert(n <= MAX_VERTICES);
    Body *b = new Body();

    b->shape.num_vertices = n;
    auto &vs = b->shape.vertices;
    float step = 2 * M_PI / n;
    for (int i = 0; i < n; ++i)
        vs[i] = size * Vec2(cosf(i * step), -sinf(i * step));

    b->pos = pos;

    b->angle = 0;
    b->mass = 1.f;
    b->inv_mass = 1.f;
    b->restitution = 0.3f;
    b->intert = 1000.f;
    b->inv_inert = 1.f / 1000.f;

    return b;
}

int main() {
    std::default_random_engine eng;
    std::uniform_real_distribution<float> angle_dist(0.f, 2 * M_PI);
    std::uniform_int_distribution<int> sides_dist(3, 8);

    std::vector<sf::Vertex> vbuffer;
    vbuffer.reserve(128);

    sf::RenderWindow window(sf::VideoMode(800, 600), "Polyphys");
    window.setFramerateLimit(60);
    using Kbd = sf::Keyboard;

    World w;
    w.iterations = 10;
    w.time_step = 1 / 60.f;

    w.bodies.push_back(static_rect(Vec2(0, 580), Vec2(800, 20)));
    w.bodies.push_back(static_rect(Vec2(0, 0), Vec2(20, 600)));
    w.bodies.push_back(static_rect(Vec2(780, 0), Vec2(20, 600)));

    const sf::Color COLOR = sf::Color(0xfc, 0x94, 0xaf);
    auto draw_line = [&vbuffer](Vec2 u, Vec2 v, const sf::Color &clr) {
        vbuffer.emplace_back(sf::Vector2f(u.x, u.y), clr);
        vbuffer.emplace_back(sf::Vector2f(v.x, v.y), clr);
    };

    while (window.isOpen()) {
        sf::Event ev;
        /* m.solve(); */
        while (window.pollEvent(ev)) {
            if (ev.type == sf::Event::Closed)
                window.close();

            if (ev.type == sf::Event::KeyPressed) {
                switch (ev.key.code) {
                case Kbd::Escape:
                    window.close();
                    break;
                default:
                    break;
                };
            }

            if (ev.type == sf::Event::MouseButtonPressed) {
                if (ev.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2i coords(ev.mouseButton.x, ev.mouseButton.y);
                    sf::Vector2f pos = window.mapPixelToCoords(coords);
                    int sides = sides_dist(eng);
                    Body *b = new_regular(Vec2(pos.x, pos.y), sides, 50.f);
                    b->angle = angle_dist(eng);
                    w.bodies.push_back(b);
                }
            }
        }

        /* const float SPD = 2.f; */
        /* if (Kbd::isKeyPressed(Kbd::Q)) */
        /*     p[idx]->angle += 0.05f; */
        /* if (Kbd::isKeyPressed(Kbd::E)) */
        /*     p[idx]->angle -= 0.05f; */
        /* if (Kbd::isKeyPressed(Kbd::A)) */
        /*     p[idx]->pos += Vec2(-SPD, 0); */
        /* if (Kbd::isKeyPressed(Kbd::D)) */
        /*     p[idx]->pos += Vec2(SPD, 0); */
        /* if (Kbd::isKeyPressed(Kbd::W)) */
        /*     p[idx]->pos += Vec2(0.f, -SPD); */
        /* if (Kbd::isKeyPressed(Kbd::S)) */
        /*     p[idx]->pos += Vec2(0.f, SPD); */

        w.update();

        vbuffer.clear();
        for (auto &b: w.bodies) {
            int nv = b->shape.num_vertices;
            auto &vs = b->shape.vertices;
            Mat22 rot(b->angle);
            Vec2 u, v;
            for (int i = 1; i < nv; ++i) {
                u = rot * vs[i - 1] + b->pos; 
                v = rot * vs[i] + b->pos;
                draw_line(u, v, COLOR);
            }
            u = rot * vs[nv - 1] + b->pos;
            v = rot * vs[0] + b->pos;
            draw_line(u, v, COLOR);

            /* for (int i = 0; i < nv; ++i) { */
            /*     Vec2 pos = rot * (vs[i] + vs[(i + 1) % nv]) / 2 + b->pos; */
            /*     Vec2 n = rot * b->shape.get_normal(i); */
            /*     u = pos; */
            /*     v = pos + 10.f * n; */
            /*     draw_line(u, v, sf::Color::Green); */
            /* } */
        }

        window.clear();
        window.draw(vbuffer.data(), vbuffer.size(), sf::Lines);
        window.display();
    }
}


