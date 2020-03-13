#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <stdbool.h>
#include <limits.h>

#define GRID_W 30
#define GRID_H 15
#define MAX_NODE_COUNT (GRID_W * GRID_H)

typedef struct Point {
    int x;
    int y;
} Point;

typedef struct Node {
    // Level data
    bool walkable;
    char graphics;

    // A* data (could be put in a separate struct for better separation of concerns)
    Point pos;
    Point came_from; // Used to recreate the path backwards. Set to {-1, -1} if not visited.
    int f_score; // Best guess at how much it will cost to go from start to goal via this node (g + h).
    int g_score; // Cost of the cheapest (known) path from start to this node.
} Node;

Node grid[GRID_W][GRID_H];

Point open_set[MAX_NODE_COUNT];
int open_set_count = 0;

void generate_maze() {
    for(int y = 0; y < GRID_H; y++) {
        for(int x = 0; x < GRID_W; x++) {
            grid[x][y].walkable =
                x == 0 ? false :
                x == GRID_W - 1 ? false :
                y == 0 ? false :
                y == GRID_H - 1 ? false :
                x == 1 && y == 1 ? true :
                x == GRID_W - 2 && y == GRID_H - 2 ? true :
                rand() % 100 > 25;

            if(grid[x][y].walkable) {
                grid[x][y].graphics = ' ';
            } else {
                grid[x][y].graphics = 'X';
            }

            grid[x][y].pos = (Point) { x, y };
            grid[x][y].came_from = (Point) { -1, -1 };
            grid[x][y].f_score = 0;
            grid[x][y].g_score = INT_MAX;
        }
    }
}

void print_maze() {
    for(int y = 0; y < GRID_H; y++) {
        for(int x = 0; x < GRID_W; x++) {
            printf("%c", grid[x][y].graphics);
        }
        printf("\n");
    }
    printf("\n");
}

bool point_eq(Point a, Point b) {
    return a.x == b.x && a.y == b.y;
}

int heuristic(Point start, Point goal) {
    return abs(goal.x - start.x) + abs(goal.y - start.y);
}

void add_to_open_set(Point p) {
    for(int i = 0; i < open_set_count; i++) {
        Point q = open_set[i];
        if(p.x == q.x && p.y == q.y) {
            return; // The point is already in the set.
        }
    }
    open_set[open_set_count++] = p;
}

Point get_point_with_lowest_f_score() {
    int lowest_f_score = INT_MAX;
    Point best = { -1, -1 };
    for(int i = 0; i < open_set_count; i++) {
        Point q = open_set[i];
        Node n = grid[q.x][q.y];
        if(n.f_score < lowest_f_score) {
            lowest_f_score = n.f_score;
            best = q;
        }
    }
    assert(best.x > -1 && best.y > -1); // Must find something.
    return best;
}

void remove_from_open_set(Point p) {
    for(int i = 0; i < open_set_count; i++) {
        Point q = open_set[i];
        if(p.x == q.x && p.y == q.y) {
            // Found it, replace with last item in array
            open_set[i] = open_set[open_set_count - 1];
            open_set_count--;
            return;
        }
    }
    printf("Failed to remove point (%d, %d) from open set.\n", p.x, p.y);
    abort();
}

int neighbours_from(Node **neighbours, Point p) {
    int count = 0;
    if(p.x > 0 && grid[p.x - 1][p.y].walkable) { neighbours[count++] = &grid[p.x - 1][p.y]; }
    if(p.y > 0 && grid[p.x][p.y - 1].walkable) { neighbours[count++] = &grid[p.x][p.y - 1]; }
    if(p.x < GRID_W - 1 && grid[p.x + 1][p.y].walkable) { neighbours[count++] = &grid[p.x + 1][p.y]; }
    if(p.y < GRID_H - 1 && grid[p.x][p.y + 1].walkable) { neighbours[count++] = &grid[p.x][p.y + 1]; }
    return count;
}

void reconstruct_path(Point start, Point goal) {
    Point pos = goal;
    while(!point_eq(pos, start)) {
        assert(pos.x > -1 && pos.y > -1);
        Node *n = &grid[pos.x][pos.y];
        if(!n->walkable) {
            printf("Warning, overriding obstacle at (%d, %d).\n", pos.x, pos.y);
        }
        n->graphics = '.';
        //printf("(%d, %d) linking back to (%d, %d)\n", pos.x, pos.y, n->came_from.x, n->came_from.y);
        pos = n->came_from;
    }

    printf("\n");
    print_maze();
}

int main() {
    srand(4);
    generate_maze();
    print_maze();

    // Goal
    Point goal = { GRID_W - 2, GRID_H - 2 };

    // Start
    Point start = { 1, 1 };
    grid[start.x][start.y].g_score = 0;
    grid[start.x][start.y].f_score = heuristic(start, goal);
    add_to_open_set(start);

    // Search
    Point pos;
    int max_steps = 9999;
    while(open_set_count > 0 && max_steps > 0) {
        pos = get_point_with_lowest_f_score();
        Node *node = &grid[pos.x][pos.y];

        if(point_eq(pos, goal)) {
            printf("Found a path to the goal.\n");
            reconstruct_path(start, goal);
            return 0;
        }

        remove_from_open_set(pos);

        Node *neighbours[4];
        int neighbours_count = neighbours_from(neighbours, pos);
        //printf("Current node (%d, %d) has %d neighbours.\n", pos.x, pos.y, neighbours_count);

        for(int i = 0; i < neighbours_count; i++) {
            Node *neighbour = neighbours[i];
            int distance_to_neighbour = 1;
            int tentative_g_score = node->g_score + distance_to_neighbour;
            //printf("  #%d at (%d, %d) Tentative g: %d\n", i, neighbour->pos.x, neighbour->pos.y, tentative_g_score);

            if(tentative_g_score < neighbour->g_score) {
                // This is a better path to this neighbour.
                neighbour->came_from = pos;
                neighbour->g_score = tentative_g_score;
                neighbour->f_score = tentative_g_score + heuristic(neighbour->pos, goal);
                add_to_open_set(neighbour->pos);
            }
        }

        max_steps--; // Bail out if it takes to many steps.
    }

    if(!max_steps) {
        printf("Bailed out./n");
    }

    printf("Failed to find a path.\n");
    abort();
}
