#include <gui/widgets/menu.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <gui/style.h>

Menu::Menu() {
}

static bool menuHeader(const char* label, const ImVec2& posMin, float menuWidth, float headerHeight, bool disabled = false) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImU32 bg = disabled ? IM_COL32(12, 40, 22, 255) : IM_COL32(16, 60, 32, 255);
    dl->AddRectFilled(posMin, ImVec2(posMin.x + menuWidth, posMin.y + headerHeight), bg, ImGui::GetStyle().FrameRounding);

    ImVec4 hdr = disabled ? ImVec4(0.05f, 0.16f, 0.08f, 1.0f) : ImVec4(0.06f, 0.24f, 0.13f, 1.0f);
    ImVec4 hov = disabled ? ImVec4(0.06f, 0.19f, 0.10f, 1.0f) : ImVec4(0.09f, 0.32f, 0.18f, 1.0f);
    ImVec4 act = disabled ? ImVec4(0.07f, 0.22f, 0.12f, 1.0f) : ImVec4(0.11f, 0.38f, 0.21f, 1.0f);
    ImVec4 txt = disabled ? ImVec4(0.62f, 0.68f, 0.62f, 1.0f) : ImVec4(0.94f, 0.98f, 0.94f, 1.0f);

    ImGui::PushFont(style::menuFont);
    ImGui::PushStyleColor(ImGuiCol_Header, hdr);
    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, hov);
    ImGui::PushStyleColor(ImGuiCol_HeaderActive, act);
    ImGui::PushStyleColor(ImGuiCol_Text, txt);
    bool open = ImGui::CollapsingHeader(label);
    ImGui::PopStyleColor(4);
    ImGui::PopFont();
    return open;
}



void Menu::registerEntry(std::string name, void (*drawHandler)(void* ctx), void* ctx, ModuleManager::Instance* inst) {
    MenuItem_t item;
    item.drawHandler = drawHandler;
    item.ctx = ctx;
    item.inst = inst;
    items[name] = item;
    if (!isInOrderList(name)) {
        MenuOption_t opt;
        opt.name = name;
        opt.open = true;
        order.push_back(opt);
    }
}

void Menu::removeEntry(std::string name) {
    items.erase(name);
}

bool Menu::draw(bool updateStates) {
    bool changed = false;
    float menuWidth = ImGui::GetContentRegionAvail().x;
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    ImGui::PushFont(style::menuFont);
    float headerHeight = ImGui::GetFrameHeight();
    float headerFontSize = ImGui::GetFontSize();
    ImGui::PopFont();
    ImVec2 checkboxOffset = ImVec2(menuWidth - headerFontSize - (6.0f * style::uiScale), - headerFontSize - (10.0f * style::uiScale));

    int displayedCount = 0;
    int rawId = 0;

    ImU32 textColor = ImGui::GetColorU32(ImGuiCol_Text);

    for (MenuOption_t& opt : order) {
        rawId++;
        if (items.find(opt.name) == items.end()) {
            continue;
        }
        if (opt.name == draggedMenuName) {
            ImGui::BeginTooltip();
            ImGui::Text("%s", draggedMenuName.c_str());
            ImGui::EndTooltip();
            continue;
        }

        // Draw dragged menu item
        if (displayedCount == insertBefore && !draggedMenuName.empty()) {
            if (updateStates) { ImGui::SetNextItemOpen(false); }
            ImVec2 posMin = ImGui::GetCursorScreenPos();
            ImVec2 posMax = ImVec2(posMin.x + menuWidth, posMin.y + headerHeight);
            style::beginDisabled();
            ImRect orignalRect = window->WorkRect;
            menuHeader((draggedMenuName + "##sdrpp_main_menu_dragging").c_str(), posMin, menuWidth, headerHeight, true);
            if (items[draggedOpt.name].inst != NULL) {
                window->WorkRect = orignalRect;
                ImVec2 pos = ImGui::GetCursorPos();
                ImGui::SetCursorPosX(pos.x + checkboxOffset.x);
                ImGui::SetCursorPosY(pos.y + checkboxOffset.y);
                bool enabled = items[draggedOpt.name].inst->isEnabled();
                ImGui::Checkbox(("##_menu_checkbox_" + draggedOpt.name).c_str(), &enabled);
                ImGui::SetCursorPos(pos);
            }
            style::endDisabled();
            window->DrawList->AddRect(posMin, posMax, textColor, 0.0f, 0, style::uiScale);
        }
        displayedCount++;

        MenuItem_t& item = items[opt.name];


        ImRect orginalRect = window->WorkRect;
        if (item.inst != NULL) {
            window->WorkRect = ImRect(orginalRect.Min, ImVec2(orginalRect.Max.x - headerFontSize - (6.0f * style::uiScale), orginalRect.Max.y));
        }

        ImVec2 posMin = ImGui::GetCursorScreenPos();
        ImVec2 posMax = ImVec2(posMin.x + menuWidth, posMin.y + headerHeight);

        headerTops[displayedCount - 1] = posMin.y;
        optionIDs[displayedCount - 1] = rawId - 1;

        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && ImGui::IsMouseHoveringRect(posMin, posMax)) {
            menuClicked = true;
            clickedMenuName = opt.name;
        }

        bool menuDragged = (menuClicked && ImGui::IsMouseDragging(ImGuiMouseButton_Left) && draggedMenuName.empty() && clickedMenuName == opt.name);
        if (menuDragged && !locked) {
            draggedMenuName = opt.name;
            draggedId = rawId - 1;
            draggedOpt = opt;
            continue;
        }
        else if (menuDragged) {
            ImGui::SetTooltip("Menu Order Locked!");
        }

        // Draw menu header and menu content. There is a lot of boilerplate because the checkbox has to be drawn before the menu, TODO: fix
        if (updateStates) { ImGui::SetNextItemOpen(opt.open); }
        if (menuHeader((opt.name + "##sdrpp_main_menu").c_str(), posMin, menuWidth, headerHeight)) {
            if (item.inst != NULL) {
                window->WorkRect = orginalRect;
                ImVec2 pos = ImGui::GetCursorPos();
                ImGui::SetCursorPosX(pos.x + checkboxOffset.x);
                ImGui::SetCursorPosY(pos.y + checkboxOffset.y);
                bool enabled = item.inst->isEnabled();
                if (ImGui::Checkbox(("##_menu_checkbox_" + opt.name).c_str(), &enabled)) {
                    enabled ? item.inst->enable() : item.inst->disable();
                    changed = true;
                }
                ImGui::SetCursorPos(pos);
            }

            // Check if the state changed
            if (!opt.open && !updateStates) {
                opt.open = true;
                changed = true;
            }

            item.drawHandler(item.ctx);
            ImGui::Spacing();
        }
        else if (item.inst != NULL) {
            window->WorkRect = orginalRect;
            ImVec2 pos = ImGui::GetCursorPos();
            ImGui::SetCursorPosX(pos.x + checkboxOffset.x);
            ImGui::SetCursorPosY(pos.y + checkboxOffset.y);
            bool enabled = item.inst->isEnabled();
            if (ImGui::Checkbox(("##_menu_checkbox_" + opt.name).c_str(), &enabled)) {
                enabled ? item.inst->enable() : item.inst->disable();
                changed = true;
            }
            ImGui::SetCursorPos(pos);

            if (opt.open && !updateStates) {
                opt.open = false;
                changed = true;
            }
        }
        else if (opt.open && !updateStates) {
            opt.open = false;
            changed = true;
        }
    }

    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left) && menuClicked) {

        if (!draggedMenuName.empty()) {
            // Move menu
            order.erase(order.begin() + draggedId);

            if (insertBefore == displayedCount) {
                order.push_back(draggedOpt);
            }
            else if (insertBeforeName != "") {
                int beforeId = 0;
                for (int i = 0; i < order.size(); i++) {
                    if (order[i].name == insertBeforeName) {
                        beforeId = i;
                        break;
                    }
                }
                order.insert(order.begin() + beforeId, draggedOpt);
            }
            changed = true;
        }

        menuClicked = false;
        draggedMenuName = "";
        insertBeforeName = "";
        insertBefore = -1;
    }

    // TODO: Figure out why the hell this is needed
    if (insertBefore == displayedCount && !draggedMenuName.empty()) {
        if (updateStates) { ImGui::SetNextItemOpen(false); }
        ImVec2 posMin = ImGui::GetCursorScreenPos();
        ImVec2 posMax = ImVec2(posMin.x + menuWidth, posMin.y + headerHeight);
        style::beginDisabled();
        ImRect orignalRect = window->WorkRect;
        menuHeader((draggedMenuName + "##sdrpp_main_menu_dragging").c_str(), posMin, menuWidth, headerHeight, true);
        if (items[draggedOpt.name].inst != NULL) {
            window->WorkRect = orignalRect;
            ImVec2 pos = ImGui::GetCursorPos();
            ImGui::SetCursorPosX(pos.x + checkboxOffset.x);
            ImGui::SetCursorPosY(pos.y + checkboxOffset.y);
            bool enabled = items[draggedOpt.name].inst->isEnabled();
            ImGui::Checkbox(("##_menu_checkbox_" + draggedOpt.name).c_str(), &enabled);
            ImGui::SetCursorPos(pos);
        }
        style::endDisabled();
        window->DrawList->AddRect(posMin, posMax, textColor, 0.0f, 0, style::uiScale);
    }

    if (!draggedMenuName.empty()) {
        insertBefore = displayedCount;
        ImVec2 mPos = ImGui::GetMousePos();
        for (int i = 0; i < displayedCount; i++) {
            if (headerTops[i] > mPos.y) {
                insertBefore = i;
                insertBeforeName = order[optionIDs[i]].name;
                break;
            }
        }
    }

    return changed;
}

bool Menu::isInOrderList(std::string name) {
    for (MenuOption_t opt : order) {
        if (opt.name == name) {
            return true;
        }
    }
    return false;
}
