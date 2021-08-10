#include "../include/problem.hpp"

#include <fstream>
#include <regex>

#include "../include/util.hpp"

Problem::Problem(std::string _instance, Graph* _G, std::mt19937* _MT,
                 Config _config_s, Config _config_g, int _num_agents,
                 int _max_timestep, int _max_comp_time)
    : instance(_instance),
      G(_G),
      MT(_MT),
      config_s(_config_s),
      config_g(_config_g),
      num_agents(_num_agents),
      max_timestep(_max_timestep),
      max_comp_time(_max_comp_time){};

Node* Problem::getStart(int i) const
{
  if (!(0 <= i && i < (int)config_s.size())) halt("invalid index");
  return config_s[i];
}

Node* Problem::getGoal(int i) const
{
  if (!(0 <= i && i < (int)config_g.size())) halt("invalid index");
  return config_g[i];
}

void Problem::halt(const std::string& msg) const
{
  std::cout << "error@Problem: " << msg << std::endl;
  this->~Problem();
  std::exit(1);
}

void Problem::warn(const std::string& msg) const
{
  std::cout << "warn@Problem: " << msg << std::endl;
}

// -------------------------------------------
// MAPF

MAPF_Instance::MAPF_Instance(const std::string& _instance)
    : Problem(_instance), instance_initialized(true)
{
  // read instance file
  std::ifstream file(instance);
  if (!file) halt("file " + instance + " is not found.");

  std::string line;
  std::smatch results;
  std::regex r_comment = std::regex(R"(#.+)");
  std::regex r_map = std::regex(R"(map_file=(.+))");
  std::regex r_agents = std::regex(R"(agents=(\d+))");
  std::regex r_seed = std::regex(R"(seed=(\d+))");
  std::regex r_random_problem = std::regex(R"(random_problem=(\d+))");
  std::regex r_well_formed = std::regex(R"(well_formed=(\d+))");
  std::regex r_max_timestep = std::regex(R"(max_timestep=(\d+))");
  std::regex r_max_comp_time = std::regex(R"(max_comp_time=(\d+))");
  std::regex r_sg = std::regex(R"((\d+),(\d+),(\d+),(\d+))");

  bool read_scen = true;
  bool well_formed = false;
  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();
    // comment
    if (std::regex_match(line, results, r_comment)) {
      continue;
    }
    // read map
    if (std::regex_match(line, results, r_map)) {
      G = new Grid(results[1].str());
      continue;
    }
    // set agent num
    if (std::regex_match(line, results, r_agents)) {
      num_agents = std::stoi(results[1].str());
      continue;
    }
    // set random seed
    if (std::regex_match(line, results, r_seed)) {
      MT = new std::mt19937(std::stoi(results[1].str()));
      continue;
    }
    // skip reading initial/goal nodes
    if (std::regex_match(line, results, r_random_problem)) {
      if (std::stoi(results[1].str())) {
        read_scen = false;
        config_s.clear();
        config_g.clear();
      }
      continue;
    }
    //
    if (std::regex_match(line, results, r_well_formed)) {
      if (std::stoi(results[1].str())) well_formed = true;
      continue;
    }
    // set max timestep
    if (std::regex_match(line, results, r_max_timestep)) {
      max_timestep = std::stoi(results[1].str());
      continue;
    }
    // set max computation time
    if (std::regex_match(line, results, r_max_comp_time)) {
      max_comp_time = std::stoi(results[1].str());
      continue;
    }
    // read initial/goal nodes
    if (std::regex_match(line, results, r_sg) && read_scen &&
        (int)config_s.size() < num_agents) {
      int x_s = std::stoi(results[1].str());
      int y_s = std::stoi(results[2].str());
      int x_g = std::stoi(results[3].str());
      int y_g = std::stoi(results[4].str());
      if (!G->existNode(x_s, y_s)) {
        halt("start node (" + std::to_string(x_s) + ", " + std::to_string(y_s) +
             ") does not exist, invalid scenario");
      }
      if (!G->existNode(x_g, y_g)) {
        halt("goal node (" + std::to_string(x_g) + ", " + std::to_string(y_g) +
             ") does not exist, invalid scenario");
      }

      Node* s = G->getNode(x_s, y_s);
      Node* g = G->getNode(x_g, y_g);
      config_s.push_back(s);
      config_g.push_back(g);
    }
  }

  // set default value not identified params
  if (MT == nullptr) MT = new std::mt19937(DEFAULT_SEED);
  if (max_timestep == 0) max_timestep = DEFAULT_MAX_TIMESTEP;
  if (max_comp_time == 0) max_comp_time = DEFAULT_MAX_COMP_TIME;

  // check starts/goals
  if (num_agents <= 0) halt("invalid number of agents");
  const int config_s_size = config_s.size();
  if (!config_s.empty() && num_agents > config_s_size) {
    warn("given starts/goals are not sufficient\nrandomly create instances");
  }
  if (num_agents > config_s_size) {
    if (well_formed) {
      setWellFormedInstance();
    } else {
      setRandomStartsGoals();
    }
  }

  // trimming
  config_s.resize(num_agents);
  config_g.resize(num_agents);
}

MAPF_Instance::MAPF_Instance(MAPF_Instance* P, Config _config_s,
                             Config _config_g, int _max_comp_time,
                             int _max_timestep)
    : Problem(P->getInstanceFileName(), P->getG(), P->getMT(), _config_s,
              _config_g, P->getNum(), _max_timestep, _max_comp_time),
      instance_initialized(false)
{
}

MAPF_Instance::MAPF_Instance(MAPF_Instance* P, int _max_comp_time)
    : Problem(P->getInstanceFileName(), P->getG(), P->getMT(),
              P->getConfigStart(), P->getConfigGoal(), P->getNum(),
              P->getMaxTimestep(), _max_comp_time),
      instance_initialized(false)
{
}

MAPF_Instance::~MAPF_Instance()
{
  if (instance_initialized) {
    if (G != nullptr) delete G;
    if (MT != nullptr) delete MT;
  }
}

void MAPF_Instance::setRandomStartsGoals()
{
  // initialize
  config_s.clear();
  config_g.clear();

  // get grid size
  Grid* grid = reinterpret_cast<Grid*>(G);
  const int N = grid->getWidth() * grid->getHeight();

  // set starts
  std::vector<int> starts(N);
  std::iota(starts.begin(), starts.end(), 0);
  std::shuffle(starts.begin(), starts.end(), *MT);
  int i = 0;
  while (true) {
    while (G->getNode(starts[i]) == nullptr) {
      ++i;
      if (i >= N) halt("number of agents is too large.");
    }
    config_s.push_back(G->getNode(starts[i]));
    if ((int)config_s.size() == num_agents) break;
    ++i;
  }

  // set goals
  std::vector<int> goals(N);
  std::iota(goals.begin(), goals.end(), 0);
  std::shuffle(goals.begin(), goals.end(), *MT);
  int j = 0;
  while (true) {
    while (G->getNode(goals[j]) == nullptr) {
      ++j;
      if (j >= N) halt("set goal, number of agents is too large.");
    }
    // retry
    if (G->getNode(goals[j]) == config_s[config_g.size()]) {
      config_g.clear();
      std::shuffle(goals.begin(), goals.end(), *MT);
      j = 0;
      continue;
    }
    config_g.push_back(G->getNode(goals[j]));
    if ((int)config_g.size() == num_agents) break;
    ++j;
  }
}

/*
 * Note: it is hard to generate well-formed instances
 * with dense situations (e.g., ≥300 agents in arena)
 */
void MAPF_Instance::setWellFormedInstance()
{
  // initialize
  config_s.clear();
  config_g.clear();

  // get grid size
  const int N = G->getNodesSize();
  Nodes prohibited, starts_goals;

  while ((int)config_g.size() < getNum()) {
    while (true) {
      // determine start
      Node* s;
      do {
        s = G->getNode(getRandomInt(0, N - 1, MT));
      } while (s == nullptr || inArray(s, prohibited));

      // determine goal
      Node* g;
      do {
        g = G->getNode(getRandomInt(0, N - 1, MT));
      } while (g == nullptr || g == s || inArray(g, prohibited));

      // ensure well formed property
      auto path = G->getPath(s, g, starts_goals);
      if (!path.empty()) {
        config_s.push_back(s);
        config_g.push_back(g);
        starts_goals.push_back(s);
        starts_goals.push_back(g);
        for (auto v : path) {
          if (!inArray(v, prohibited)) prohibited.push_back(v);
        }
        break;
      }
    }
  }
}

void MAPF_Instance::makeScenFile(const std::string& output_file)
{
  Grid* grid = reinterpret_cast<Grid*>(G);
  std::ofstream log;
  log.open(output_file, std::ios::out);
  log << "map_file=" << grid->getMapFileName() << "\n";
  log << "agents=" << num_agents << "\n";
  log << "seed=0\n";
  log << "random_problem=0\n";
  log << "max_timestep=" << max_timestep << "\n";
  log << "max_comp_time=" << max_comp_time << "\n";
  for (int i = 0; i < num_agents; ++i) {
    log << config_s[i]->pos.x << "," << config_s[i]->pos.y << ","
        << config_g[i]->pos.x << "," << config_g[i]->pos.y << "\n";
  }
  log.close();
}

// -------------------------------------------
// MAPD
MAPD_Instance::MAPD_Instance(const std::string& _instance)
    : Problem(_instance), current_timestep(-1), specify_pickup_deliv_locs(true)
{
  // read instance file
  std::ifstream file(instance);
  if (!file) halt("file " + instance + " is not found.");

  std::string line;
  std::smatch results;
  std::regex r_comment = std::regex(R"(#.+)");
  std::regex r_map = std::regex(R"(map_file=(.+))");
  std::regex r_agents = std::regex(R"(agents=(\d+))");
  std::regex r_seed = std::regex(R"(seed=(\d+))");
  std::regex r_max_timestep = std::regex(R"(max_timestep=(\d+))");
  std::regex r_max_comp_time = std::regex(R"(max_comp_time=(\d+))");
  std::regex r_task_num = std::regex(R"(task_num=(\d+))");
  std::regex r_task_frequency = std::regex(R"(task_frequency=(.+))");
  std::regex r_specify_pikup_deliv_locs =
      std::regex(R"(specify_pikup_deliv_locs=(\d+))");
  std::regex r_sg = std::regex(R"((\d+),(\d+))");

  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();
    // comment
    if (std::regex_match(line, results, r_comment)) {
      continue;
    }
    // read map
    if (std::regex_match(line, results, r_map)) {
      G = new Grid(results[1].str());
      continue;
    }
    // set agent num
    if (std::regex_match(line, results, r_agents)) {
      num_agents = std::stoi(results[1].str());
      continue;
    }
    // set random seed
    if (std::regex_match(line, results, r_seed)) {
      MT = new std::mt19937(std::stoi(results[1].str()));
      continue;
    }
    // set max timestep
    if (std::regex_match(line, results, r_max_timestep)) {
      max_timestep = std::stoi(results[1].str());
      continue;
    }
    // set max computation time
    if (std::regex_match(line, results, r_max_comp_time)) {
      max_comp_time = std::stoi(results[1].str());
      continue;
    }
    // set the number of tasks
    if (std::regex_match(line, results, r_task_num)) {
      task_num = std::stoi(results[1].str());
      continue;
    }
    // set task frequency
    if (std::regex_match(line, results, r_task_frequency)) {
      task_frequency = std::stof(results[1].str());
      continue;
    }
    // set task frequency
    if (std::regex_match(line, results, r_specify_pikup_deliv_locs)) {
      specify_pickup_deliv_locs = (bool)std::stoi(results[1].str());
      continue;
    }
    // read initial nodes
    if (std::regex_match(line, results, r_sg) &&
        (int)config_s.size() < num_agents) {
      int x_s = std::stoi(results[1].str());
      int y_s = std::stoi(results[2].str());
      if (!G->existNode(x_s, y_s)) {
        halt("start node (" + std::to_string(x_s) + ", " + std::to_string(y_s) +
             ") does not exist, invalid scenario");
      }

      Node* s = G->getNode(x_s, y_s);
      config_s.push_back(s);
    }
  }

  // set default value not identified params
  if (MT == nullptr) MT = new std::mt19937(DEFAULT_SEED);
  if (max_timestep == 0) max_timestep = DEFAULT_MAX_TIMESTEP;
  if (max_comp_time == 0) max_comp_time = DEFAULT_MAX_COMP_TIME;
  if (task_frequency == 0) task_frequency = DEFAULT_TASK_FREQUENCY;
  if (task_num == 0) task_num = DEFAULT_TASK_NUM;

  if (specify_pickup_deliv_locs) setupSpetialNodes();
  if (LOCS_PICKUP.empty()) {
    LOCS_PICKUP = G->getV();
    LOCS_DELIVERY = G->getV();
  }

  // check starts
  if (num_agents <= 0) halt("invalid number of agents");
  if (num_agents > (int)config_s.size()) {
    if (!config_s.empty()) {
      warn("given starts are not sufficient\nrandomly create instances");
      config_s.clear();
    }

    // set starts
    std::vector<int> starts(G->getNodesSize());
    std::iota(starts.begin(), starts.end(), 0);
    if (specify_pickup_deliv_locs && !LOCS_NONTASK_ENDPOINTS.empty()) {
      starts.clear();
      for (auto v : LOCS_NONTASK_ENDPOINTS) starts.push_back(v->id);
    }
    std::shuffle(starts.begin(), starts.end(), *MT);

    int i = 0;
    while (true) {
      while (G->getNode(starts[i]) == nullptr) {
        ++i;
        if (i >= (int)starts.size()) halt("number of agents is too large.");
      }

      config_s.push_back(G->getNode(starts[i]));
      if ((int)config_s.size() == num_agents) break;
      ++i;
    }
  }

  // trimming
  config_s.resize(num_agents);

  // initialize
  update();
}

MAPD_Instance::~MAPD_Instance()
{
  for (auto task : TASKS_OPEN) delete task;
  for (auto task : TASKS_CLOSED) delete task;
}

void MAPD_Instance::setupSpetialNodes()
{
  Grid* grid = reinterpret_cast<Grid*>(G);

  // read instance file
#ifdef _MAPDIR_
  std::ifstream file(_MAPDIR_ + grid->getMapFileName() + ".pd");
#else
  std::ifstream file(grid->getMapFileName() + ".pd");
#endif
  if (!file) return;

  std::string line, s;
  std::smatch results;
  std::regex r_obj = std::regex(R"([@T])");
  std::regex r_pickup = std::regex(R"([psa])");  // pickup loc.
  std::regex r_deliv = std::regex(R"([dsa])");   // delivery loc.
  std::regex r_end = std::regex(R"([ea])");      // end loc.

  const int width = grid->getWidth();

  int y = 0;
  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();

    if ((int)line.size() != width) halt("pd format is invalid");

    for (int x = 0; x < width; ++x) {
      if (!G->existNode(x, y)) continue;

      auto v = G->getNode(x, y);
      s = line[x];
      bool flg_endpoints = false;
      if (std::regex_match(s, results, r_pickup)) {
        LOCS_PICKUP.push_back(v);
        flg_endpoints = true;
      }
      if (std::regex_match(s, results, r_deliv)) {
        LOCS_DELIVERY.push_back(v);
        flg_endpoints = true;
      }
      if (std::regex_match(s, results, r_end)) {
        LOCS_NONTASK_ENDPOINTS.push_back(v);
        flg_endpoints = true;
      }
      if (flg_endpoints) {
        LOCS_ENDPOINTS.push_back(v);
      }
    }
    ++y;
  }
}

void MAPD_Instance::update()
{
  // check finished tasks
  auto itr = TASKS_OPEN.begin();
  while (itr != TASKS_OPEN.end()) {
    auto task = *itr;

    // not at delivery location
    if (task->loc_current != task->loc_delivery) {
      ++itr;
      continue;
    }

    task->timestep_finished = current_timestep + 1;
    TASKS_CLOSED.push_back(task);

    // remove from OPEN list
    itr = TASKS_OPEN.erase(itr);
  }

  // create new tasks
  int created_task_num = int(TASKS_OPEN.size() + TASKS_CLOSED.size());
  if (created_task_num < task_num) {
    int new_task_num = (int)task_frequency;
    if (task_frequency < 1 && getRandomFloat(0, 1, MT) < task_frequency) {
      new_task_num = 1;
    }
    new_task_num = std::min(new_task_num, task_num - created_task_num);
    for (int i = 0; i < new_task_num; ++i) {
      Node *p, *d;
      do {
        p = randomChoose(LOCS_PICKUP, MT);
        d = randomChoose(LOCS_DELIVERY, MT);
      } while (p == d);
      TASKS_OPEN.push_back(new Task(p, d, current_timestep + 1));
    }
  }

  // update timestep
  ++current_timestep;
};
