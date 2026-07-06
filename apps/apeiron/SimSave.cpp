#include "SimSave.h"
#include "CislunarMFD.h"

#include <toml++/toml.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

// ---------------------------------------------------------------------------
// Helpers: pack / unpack glm types into toml arrays.
// ---------------------------------------------------------------------------
static toml::array vec3ToToml(const glm::dvec3& v)
{
    toml::array a;
    a.push_back(v.x); a.push_back(v.y); a.push_back(v.z);
    return a;
}

static toml::array quatToToml(const glm::dquat& q)
{
    toml::array a;
    // Store as w,x,y,z (GLM memory layout is x,y,z,w — be explicit).
    a.push_back(q.w); a.push_back(q.x); a.push_back(q.y); a.push_back(q.z);
    return a;
}

static glm::dvec3 tomlToVec3(const toml::array* a)
{
    glm::dvec3 v(0.0);
    if (!a || a->size() < 3) return v;
    v.x = a->get(0)->value_or(0.0);
    v.y = a->get(1)->value_or(0.0);
    v.z = a->get(2)->value_or(0.0);
    return v;
}

static glm::dquat tomlToQuat(const toml::array* a)
{
    glm::dquat q(1.0, 0.0, 0.0, 0.0);
    if (!a || a->size() < 4) return q;
    q.w = a->get(0)->value_or(1.0);
    q.x = a->get(1)->value_or(0.0);
    q.y = a->get(2)->value_or(0.0);
    q.z = a->get(3)->value_or(0.0);
    return q;
}

// ---------------------------------------------------------------------------
// Helpers: build / parse a [transfer_plan] table in isolation.
// ---------------------------------------------------------------------------
static toml::table planToToml(const TransferPlanSnapshot& snap)
{
    toml::table tp;
    const auto& p = snap.params;
    tp.insert("departure_body", p.departureBody);
    tp.insert("arrival_body",   p.arrivalBody);
    tp.insert("central_body",   p.centralBody);
    tp.insert("t0",             p.t0);
    tp.insert("t1",             p.t1);
    tp.insert("tof_min",        p.tofMin);
    tp.insert("tof_max",        p.tofMax);
    tp.insert("n_dep",          p.nDep);
    tp.insert("n_tof",          p.nTof);
    tp.insert("sel_dep",        snap.selDep);
    tp.insert("sel_tof",        snap.selTof);
    tp.insert("park_idx",       snap.parkIdx);
    return tp;
}

static void planFromToml(const toml::table& tp, TransferPlanSnapshot& snap)
{
    snap.valid = true;
    auto& p = snap.params;
    p.departureBody = tp["departure_body"].value_or(399);
    p.arrivalBody   = tp["arrival_body"]  .value_or(4);
    p.centralBody   = tp["central_body"]  .value_or(10);
    p.t0            = tp["t0"]            .value_or(0.0);
    p.t1            = tp["t1"]            .value_or(0.0);
    p.tofMin        = tp["tof_min"]       .value_or(0.0);
    p.tofMax        = tp["tof_max"]       .value_or(0.0);
    p.nDep          = tp["n_dep"]         .value_or(80);
    p.nTof          = tp["n_tof"]         .value_or(60);
    snap.selDep     = tp["sel_dep"]       .value_or(-1);
    snap.selTof     = tp["sel_tof"]       .value_or(-1);
    snap.parkIdx    = tp["park_idx"]      .value_or(0);
}

// ---------------------------------------------------------------------------
// Helpers: build / parse a [cislunar_plan] table.
// ---------------------------------------------------------------------------
static toml::table cislunarPlanToToml(const CislunarPlanSnapshot& snap)
{
    toml::table tp;
    const auto& p = snap.params;
    tp.insert("t0",      p.t0);
    tp.insert("t1",      p.t1);
    tp.insert("tof_min", p.tofMin);
    tp.insert("tof_max", p.tofMax);
    tp.insert("n_dep",   p.nDep);
    tp.insert("n_tof",   p.nTof);
    tp.insert("dep_r",   vec3ToToml(p.departureR));
    tp.insert("dep_v",   vec3ToToml(p.departureV));
    tp.insert("sel_dep", snap.selDep);
    tp.insert("sel_tof", snap.selTof);
    tp.insert("park_idx",   snap.parkIdx);
    tp.insert("loi_pe_idx", snap.loiPeIdx);
    return tp;
}

static void cislunarPlanFromToml(const toml::table& tp, CislunarPlanSnapshot& snap)
{
    snap.valid = true;
    auto& p = snap.params;
    p.t0     = tp["t0"]     .value_or(0.0);
    p.t1     = tp["t1"]     .value_or(0.0);
    p.tofMin = tp["tof_min"].value_or(0.0);
    p.tofMax = tp["tof_max"].value_or(0.0);
    p.nDep   = tp["n_dep"]  .value_or(90);
    p.nTof   = tp["n_tof"]  .value_or(40);
    // Fixed for cislunar — always Earth-Moon.
    p.centralBody          = 399;
    p.arrivalBody          = 301;
    p.departureBody        = 399;
    p.muCentral            = 398600.4418;
    p.useDepartureOverride = true;
    p.departureR = tomlToVec3(tp["dep_r"].as_array());
    p.departureV = tomlToVec3(tp["dep_v"].as_array());
    snap.selDep   = tp["sel_dep"]   .value_or(-1);
    snap.selTof   = tp["sel_tof"]   .value_or(-1);
    snap.parkIdx  = tp["park_idx"]  .value_or(1);
    snap.loiPeIdx = tp["loi_pe_idx"].value_or(0);
}

// ---------------------------------------------------------------------------
// Helpers: build / parse a [gateway_plan] table.
// ---------------------------------------------------------------------------
static toml::table gatewayPlanToToml(const GatewayPlanSnapshot& snap)
{
    toml::table tp;
    const auto& p = snap.params;
    tp.insert("t0",      p.t0);
    tp.insert("t1",      p.t1);
    tp.insert("tof_min", p.tofMin);
    tp.insert("tof_max", p.tofMax);
    tp.insert("n_dep",   p.nDep);
    tp.insert("n_tof",   p.nTof);
    tp.insert("dep_r",   vec3ToToml(p.departureR));
    tp.insert("dep_v",   vec3ToToml(p.departureV));
    tp.insert("sel_dep", snap.selDep);
    tp.insert("sel_tof", snap.selTof);
    tp.insert("park_idx", snap.parkIdx);
    return tp;
}

static void gatewayPlanFromToml(const toml::table& tp, GatewayPlanSnapshot& snap)
{
    snap.valid = true;
    auto& p = snap.params;
    p.t0     = tp["t0"]     .value_or(0.0);
    p.t1     = tp["t1"]     .value_or(0.0);
    p.tofMin = tp["tof_min"].value_or(0.0);
    p.tofMax = tp["tof_max"].value_or(0.0);
    p.nDep   = tp["n_dep"]  .value_or(90);
    p.nTof   = tp["n_tof"]  .value_or(40);
    // Fixed for Earth→Gateway — arrival override fn is re-wired in restorePlan.
    p.centralBody          = 399;
    p.arrivalBody          = 301;
    p.departureBody        = 399;
    p.muCentral            = 398600.4418;
    p.useDepartureOverride = true;
    p.departureR = tomlToVec3(tp["dep_r"].as_array());
    p.departureV = tomlToVec3(tp["dep_v"].as_array());
    snap.selDep  = tp["sel_dep"] .value_or(-1);
    snap.selTof  = tp["sel_tof"] .value_or(-1);
    snap.parkIdx = tp["park_idx"].value_or(1);
}

// ---------------------------------------------------------------------------
bool saveSimState(const std::string& path, const SimSaveData& data)
{
    try {
        std::filesystem::create_directories(
            std::filesystem::path(path).parent_path());

        toml::table root;

        // ---- [sim] ----
        toml::table sim;
        sim.insert("elapsed_s",    data.simElapsed);
        sim.insert("speed_target", data.simSpeedTarget);
        if (!data.refBodyName.empty())
            sim.insert("ref_body", data.refBodyName);
        root.insert("sim", std::move(sim));

        // ---- [[spacecraft]] ----
        toml::array scArr;
        for (const auto& sc : data.spacecraft) {
            toml::table t;
            t.insert("position",        vec3ToToml(sc.position));
            t.insert("velocity",        vec3ToToml(sc.velocity));
            t.insert("attitude",        quatToToml(sc.attitude));
            t.insert("angular_velocity",vec3ToToml(sc.angularVelocity));
            scArr.push_back(std::move(t));
        }
        root.insert("spacecraft", std::move(scArr));

        // ---- [transfer_plan] ----
        if (data.plan.valid)
            root.insert("transfer_plan", planToToml(data.plan));

        // ---- [cislunar_plan] ----
        if (data.cislunarPlan.valid)
            root.insert("cislunar_plan", cislunarPlanToToml(data.cislunarPlan));

        // ---- [gateway_plan] ----
        if (data.gatewayPlan.valid)
            root.insert("gateway_plan", gatewayPlanToToml(data.gatewayPlan));

        // ---- [docking] ----
        if (data.docking.active) {
            toml::table dk;
            dk.insert("active_sc",    data.docking.activeScIdx);
            dk.insert("passive_sc",   data.docking.passiveScIdx);
            dk.insert("active_port",  data.docking.activePortIdx);
            dk.insert("passive_port", data.docking.passivePortIdx);
            root.insert("docking", std::move(dk));
        }

        // ---- [[events]] ----
        if (!data.events.empty()) {
            toml::array evArr;
            for (const auto& e : data.events) {
                toml::table t;
                t.insert("name",         e.name);
                t.insert("event_et",     e.eventET);
                t.insert("ann10",        e.ann10min);
                t.insert("ann5",         e.ann5min);
                t.insert("ann1",         e.ann1min);
                t.insert("ann10s",       e.ann10s);
                t.insert("ann0",         e.ann0);
                t.insert("countdown10s", e.countdown10s);
                evArr.push_back(std::move(t));
            }
            root.insert("events", std::move(evArr));
        }

        std::ofstream ofs(path);
        if (!ofs) {
            std::cerr << "[SimSave] Cannot open '" << path << "' for writing\n";
            return false;
        }
        ofs << root;
        std::cout << "[SimSave] Saved to " << path << "\n";
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[SimSave] Save failed: " << e.what() << "\n";
        return false;
    }
}

// ---------------------------------------------------------------------------
bool loadSimState(const std::string& path, SimSaveData& data)
{
    try {
        auto tbl = toml::parse_file(path);

        // ---- [sim] ----
        data.simElapsed     = tbl["sim"]["elapsed_s"]   .value_or(0.0);
        data.simSpeedTarget = tbl["sim"]["speed_target"].value_or(1.0);
        data.refBodyName    = tbl["sim"]["ref_body"]    .value_or(std::string{});

        // ---- [[spacecraft]] ----
        data.spacecraft.clear();
        if (auto* arr = tbl["spacecraft"].as_array()) {
            for (auto& elem : *arr) {
                if (auto* t = elem.as_table()) {
                    SimSaveData::ScState sc;
                    sc.position        = tomlToVec3((*t)["position"]        .as_array());
                    sc.velocity        = tomlToVec3((*t)["velocity"]        .as_array());
                    sc.attitude        = tomlToQuat((*t)["attitude"]        .as_array());
                    sc.angularVelocity = tomlToVec3((*t)["angular_velocity"].as_array());
                    data.spacecraft.push_back(sc);
                }
            }
        }

        // ---- [transfer_plan] ----
        data.plan = {};
        if (auto* tp = tbl["transfer_plan"].as_table())
            planFromToml(*tp, data.plan);

        // ---- [cislunar_plan] ----
        data.cislunarPlan = {};
        if (auto* tp = tbl["cislunar_plan"].as_table())
            cislunarPlanFromToml(*tp, data.cislunarPlan);

        // ---- [gateway_plan] ----
        data.gatewayPlan = {};
        if (auto* tp = tbl["gateway_plan"].as_table())
            gatewayPlanFromToml(*tp, data.gatewayPlan);

        // ---- [docking] ----
        data.docking = {};
        if (auto* dk = tbl["docking"].as_table()) {
            data.docking.active         = true;
            data.docking.activeScIdx    = (*dk)["active_sc"]   .value_or(0);
            data.docking.passiveScIdx   = (*dk)["passive_sc"]  .value_or(1);
            data.docking.activePortIdx  = (*dk)["active_port"] .value_or(0);
            data.docking.passivePortIdx = (*dk)["passive_port"].value_or(0);
        }

        // ---- [[events]] ----
        data.events.clear();
        if (auto* arr = tbl["events"].as_array()) {
            for (auto& elem : *arr) {
                if (auto* t = elem.as_table()) {
                    SimSaveData::SavedEvent e;
                    e.name    = (*t)["name"]     .value_or(std::string{});
                    e.eventET = (*t)["event_et"] .value_or(0.0);
                    e.ann10min     = (*t)["ann10"]        .value_or(false);
                    e.ann5min      = (*t)["ann5"]         .value_or(false);
                    e.ann1min      = (*t)["ann1"]         .value_or(false);
                    e.ann10s       = (*t)["ann10s"]       .value_or(false);
                    e.ann0         = (*t)["ann0"]         .value_or(false);
                    e.countdown10s = (*t)["countdown10s"] .value_or(false);
                    data.events.push_back(e);
                }
            }
        }

        std::cout << "[SimSave] Loaded from " << path << "\n";
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[SimSave] Load failed: " << e.what() << "\n";
        return false;
    }
}
