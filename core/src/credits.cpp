// rg/credits.cpp — see credits.h.
#include "rg/credits.h"

#include "json_util.h"

#include <algorithm>
#include <set>
#include <sstream>

namespace rg {

namespace {

using detail::json;

bool fail(std::string* err, const std::string& origin, const std::string& message) {
    if (err != nullptr) *err = origin + ": " + message;
    return false;
}

bool kind_from_name(const std::string& s, CreditKind& out) {
    if (s == "data") out = CreditKind::Data;
    else if (s == "service") out = CreditKind::Service;
    else if (s == "software") out = CreditKind::Software;
    else if (s == "assets") out = CreditKind::Assets;
    else return false;
    return true;
}

std::string trim(std::string s) {
    const auto not_space = [](unsigned char c) { return c != ' ' && c != '\t' && c != '\r'; };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

} // namespace

const char* credit_kind_name(CreditKind kind) {
    switch (kind) {
        case CreditKind::Data: return "data";
        case CreditKind::Service: return "service";
        case CreditKind::Software: return "software";
        case CreditKind::Assets: return "assets";
    }
    return "data";
}

std::optional<Credits> parse_credits(const std::string& json_text, const std::string& origin, std::string* err) {
    const std::optional<json> parsed = detail::parse_json(json_text);
    if (!parsed || !parsed->is_object()) {
        fail(err, origin, "not a JSON object");
        return std::nullopt;
    }
    Credits out;
    if (!detail::get_string(*parsed, "format", out.format) || out.format != "rg.credits/1") {
        fail(err, origin, "\"format\" must be \"rg.credits/1\"");
        return std::nullopt;
    }
    const auto entries = parsed->find("entries");
    if (entries == parsed->end() || !entries->is_array()) {
        fail(err, origin, "\"entries\" must be an array");
        return std::nullopt;
    }
    std::set<std::string> ids;
    std::size_t index = 0;
    for (const json& e : *entries) {
        const std::string where = "entries[" + std::to_string(index++) + "]";
        if (!e.is_object()) {
            fail(err, origin, where + " is not an object");
            return std::nullopt;
        }
        CreditEntry c;
        std::string kind;
        if (!detail::get_string(e, "id", c.id) || c.id.empty()) {
            fail(err, origin, where + ": \"id\" missing");
            return std::nullopt;
        }
        if (!ids.insert(c.id).second) {
            fail(err, origin, where + ": duplicate id \"" + c.id + "\"");
            return std::nullopt;
        }
        if (!detail::get_string(e, "kind", kind) || !kind_from_name(kind, c.kind)) {
            fail(err, origin, where + " (" + c.id + "): \"kind\" must be data, service, software or assets");
            return std::nullopt;
        }
        if (!detail::get_string(e, "name", c.name) || c.name.empty() || !detail::get_string(e, "provider", c.provider) ||
            !detail::get_string(e, "licence", c.licence) || c.licence.empty()) {
            fail(err, origin, where + " (" + c.id + "): \"name\", \"provider\" and \"licence\" are required strings");
            return std::nullopt;
        }
        detail::get_string(e, "licence_url", c.licence_url);
        detail::get_string(e, "attribution_text", c.attribution_text);
        detail::get_string(e, "note", c.note);
        if (!detail::get_bool(e, "attribution_required", c.attribution_required)) {
            fail(err, origin, where + " (" + c.id + "): \"attribution_required\" (bool) is required");
            return std::nullopt;
        }
        if (c.attribution_required && c.attribution_text.empty()) {
            fail(err, origin, where + " (" + c.id + "): attribution is required but attribution_text is empty");
            return std::nullopt;
        }
        detail::get_bool(e, "in_game", c.in_game);
        detail::get_bool(e, "on_attribution_line", c.on_attribution_line);
        if (c.on_attribution_line && c.attribution_text.empty()) {
            fail(err, origin, where + " (" + c.id + "): on_attribution_line needs an attribution_text");
            return std::nullopt;
        }
        if (const auto d = e.find("geo2map_datasets"); d != e.end()) {
            if (!d->is_array()) {
                fail(err, origin, where + " (" + c.id + "): \"geo2map_datasets\" must be an array of strings");
                return std::nullopt;
            }
            for (const json& s : *d) {
                if (!s.is_string()) {
                    fail(err, origin, where + " (" + c.id + "): \"geo2map_datasets\" must be an array of strings");
                    return std::nullopt;
                }
                c.geo2map_datasets.push_back(s.get<std::string>());
            }
        }
        out.entries.push_back(std::move(c));
    }
    return out;
}

std::optional<Credits> load_credits(const std::string& path, std::string* err) {
    const std::optional<std::string> text = detail::read_text_file(path);
    if (!text) {
        fail(err, path, "cannot read file");
        return std::nullopt;
    }
    return parse_credits(*text, path, err);
}

std::string attribution_line(const Credits& credits) {
    std::string line;
    for (const CreditEntry& e : credits.entries) {
        if (!e.on_attribution_line || !e.in_game) continue;
        if (!line.empty()) line += " | ";
        line += e.attribution_text;
    }
    return line;
}

std::vector<CreditsSection> credits_sections(const Credits& credits) {
    struct Spec {
        const char* title;
        CreditKind kind;
        bool in_game;
    };
    static const Spec specs[] = {
        {"Map and terrain data", CreditKind::Data, true},
        {"Services", CreditKind::Service, true},
        {"Software", CreditKind::Software, true},
        {"Assets", CreditKind::Assets, true},
        {"Prepared for upcoming features (not used in the game yet)", CreditKind::Data, false},
    };
    std::vector<CreditsSection> out;
    for (const Spec& s : specs) {
        CreditsSection section;
        section.title = s.title;
        for (const CreditEntry& e : credits.entries) {
            const bool match = s.in_game ? (e.in_game && e.kind == s.kind) : !e.in_game;
            if (match) section.entries.push_back(&e);
        }
        if (!section.entries.empty()) out.push_back(std::move(section));
    }
    return out;
}

std::vector<std::string> parse_geo2map_dataset_names(const std::string& md) {
    std::vector<std::string> names;
    std::istringstream in(md);
    std::string line;
    bool in_table = false;
    while (std::getline(in, line)) {
        const std::string t = trim(line);
        if (!in_table) {
            if (t.rfind("| Dataset |", 0) == 0) in_table = true;
            continue;
        }
        if (t.empty() || t[0] != '|') break;                       // table ended
        if (t.rfind("|---", 0) == 0 || t.rfind("| ---", 0) == 0) continue; // separator row
        const std::size_t end = t.find('|', 1);
        if (end == std::string::npos) continue;
        names.push_back(trim(t.substr(1, end - 1)));
    }
    return names;
}

std::vector<std::string> uncovered_geo2map_datasets(const Credits& credits, const std::vector<std::string>& datasets) {
    std::set<std::string> covered;
    for (const CreditEntry& e : credits.entries) covered.insert(e.geo2map_datasets.begin(), e.geo2map_datasets.end());
    std::vector<std::string> missing;
    for (const std::string& d : datasets) {
        if (covered.count(d) == 0) missing.push_back(d);
    }
    return missing;
}

} // namespace rg
