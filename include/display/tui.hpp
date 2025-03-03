#ifndef TUI_HPP
#define TUI_HPP

#include <ftxui/dom/elements.hpp>  // for color, Fit, LIGHT, align_right, bold, DOUBLE
#include <ftxui/dom/table.hpp>      // for Table, TableSelection
#include <ftxui/screen/screen.hpp>  // for Screen
#include <iostream>                 // for endl, cout, ostream
#include <string>                   // for basic_string, allocator, string
#include <vector>                   // for vector

#include "ftxui/dom/node.hpp"  // for Render
#include "ftxui/screen/color.hpp"  // for Color, Color::Blue, Color::Cyan, Color::White, ftxui
#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>

#include "definitions.h"
#include "queueTemplate/queueTemplates.hpp"

//formerly doubleSpinBox_1

// displayTable::displayTable() : screen_(ftxui::ScreenInteractive::TerminalOutput()) {}

// void displayTable::Run() {
// screen_.Loop(ftxui::Component:   From(ftxui::Container::Vertical({
//     // Add your components here
// })));
// }


class displayTable : public spscConsumer<dataStructures1::dataTuple>
{

public:
    using spscConsumer<dataStructures1::dataTuple>::spscConsumer;


    using spscConsumer<dataStructures1::dataTuple>::start;
    using spscConsumer<dataStructures1::dataTuple>::stop;

protected:
    using spscConsumer<dataStructures1::dataTuple>::running;
    using spscConsumer<dataStructures1::dataTuple>::q;
    using spscConsumer<dataStructures1::dataTuple>::monitor;

    std::string reset_position;

    void consume(const dataStructures1::dataTuple& valTuple) override
    {
        // std::cout << "TUI update" <<std::endl;
        // std::cout << "Thread: " << std::this_thread::get_id() << std::endl;

        uint64_t iterations = std::get<0>(valTuple);
        auto aiIn = std::get<1>(valTuple);
        auto aoIn = std::get<2>(valTuple);
        auto roIn = std::get<3>(valTuple);
        auto calcIn = std::get<4>(valTuple);

        //uint32_t length = 8;
        using namespace ftxui;

        std::vector<std::vector<std::string>> tabledata(DISPLAY_CHANNELS+1, std::vector<std::string>(5));
        //tabledata[0][0] = "Channel";
        tabledata[0][1] = "ai Values";
        tabledata[0][2] = "ao Values";
        tabledata[0][3] = "ro Values";
        tabledata[0][4] = "calc Values";
        for (int j=0; j<DISPLAY_CHANNELS; j++){
            tabledata[j+1][0] = std::to_string(j);
        }

        tabledata[0][0] = std::to_string(iterations);
        for (int i=0; i<INPUT_CHANNELS; i++){
            tabledata[i+1][1] = std::to_string(aiIn[i]);
        }
        for (int i=0; i<OUTPUT_CHANNELS; i++){
            tabledata[i+1][2] = std::to_string(aoIn[i]);
        }
        for (int i=0; i<RELAY_CHANNELS; i++){
            tabledata[i+1][3] = std::to_string(roIn[i]);
        }
        for (int i=0; i<CALC_CHANNELS+INPUT_CHANNELS; i++){
            tabledata[i+1][4] = std::to_string(calcIn[i]);
        }


        auto table = Table(tabledata);


        table.SelectAll().Border(LIGHT);
        

        // Add border around the first column.
        table.SelectColumn(0).Border(LIGHT);
        table.SelectRows(0,DISPLAY_CHANNELS-1).SeparatorVertical(LIGHT);
        table.SelectColumn(0).DecorateCells(align_right);

        // Make first row bold with a double border.
        table.SelectRow(0).Decorate(bold);
        table.SelectRow(0).SeparatorVertical(LIGHT);
        table.SelectRow(0).Border(DOUBLE);


        // Select row from the second to the last.
        auto content = table.SelectRows(1, -1);
        // Alternate in between 3 colors.
        content.DecorateCellsAlternateRow(color(Color::Blue), 3, 0);
        content.DecorateCellsAlternateRow(color(Color::Cyan), 3, 1);
        content.DecorateCellsAlternateRow(color(Color::White), 3, 2);

        
        auto document = table.Render();
        auto screen = Screen::Create(Dimension::Fit(document));
        Render(screen, document);
        std::cout << reset_position;
        screen.Print();
        reset_position = screen.ResetPosition(); //allows redrawing
        //std::cout << std::endl;

    }

};

#endif