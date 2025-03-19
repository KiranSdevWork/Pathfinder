#include "widget.h"

#include <QApplication>
#include <QAbstractTableModel>
#include <QTableView>
#include <vector>
#include <QWidget>
#include <QHBoxLayout>
#include <QPushButton>
#include <QFileDialog>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDebug>
#include <QJsonArray>
#include <cmath>
#include <queue>
#include <QRandomGenerator> // For test

using namespace std;
//------------------------//
//constant used for max size error check within the application (program is stress tested for upto 200x200, but currently capped for 50
constexpr int constMAXSIZE = 50;

//Width and Height of the central data grid used in algorithm, ideally this wont be global scope, but will be part of Data Model
// For the short time development these tqo parameters are global in scope
int nWd = -1;
int nHt = -1;

//CPosition struct is used for 0 based location in a data structure
struct CPosition{
    int m_nXPos, m_nYPos;
    CPosition(int _x, int _y) : m_nXPos(_x), m_nYPos(_y) {}
    bool operator==(const CPosition& other) const { return ((m_nXPos == other.m_nXPos) && (m_nYPos == other.m_nYPos)); }
    CPosition& operator=(const CPosition& pos)
    {
        // Check for self-assignment
        if (this != &pos) {
            m_nXPos = pos.m_nXPos;
            m_nYPos = pos.m_nYPos;
        }
        return *this;
    }
};

//CGridItem is the main data structure behind each cell of a data grid
// This also includes parameters to the A* search algorithm as plan is to show all distance data on each explored cell item (Time permits!)
// This is a central element on which path finding algorithm is implemented
struct CGridItem
{
    CPosition   m_pos;
    CPosition   m_Parent;
    int distTotal, distFromStart, distEstToGoal; // Values used by the A* algorithm
    int status; // 0 - default, 1 - open, -1 = close
    int nGridValue; // Read data for cell

    CGridItem(int _x=0, int _y=0, int _nVal=1) : m_pos(_x,_y), m_Parent(0,0), distTotal(0), distFromStart(0), distEstToGoal(0), status(0), nGridValue(_nVal) {}

    // Overload comparison operators for priority queue
    bool operator>(const CGridItem& other) const {    return distTotal > other.distTotal; }
};

//FindPath() is the central A* algorithm used here for path finding
// Inputs : grid - data source, width & height of the grid, start & goal position
// First input parameters are validated - ideally they will be separated & even this algorithm will be part of the class structure (proposal attached in the repo)
// This algo uses Manhattan method (i.e. no diagonal), but can be extended for diagonal search (Euclidian, etc...)
bool FindPath( vector<vector<CGridItem>>& grid, int nWd, int nHt, CPosition& start,  CPosition& goal)
{
    // Define possible movements (4 directions: up, down, left, right)
    const int directionX[] = {-1, 0, 1, 0};
    const int directionY[] = {0, 1, 0, -1};
    qDebug() << "...........FindPath().............." ;
    if(start == goal)
    {
        qDebug() << "...........start & goal are same.............." ;
        return true;
    }
    if((start.m_nXPos < 0) || (start.m_nYPos < 0) || (start.m_nXPos == nWd) || (start.m_nYPos == nHt))
    {
        qDebug() << "...........Out of bound start.............." ;
        return false;
    }
    if(grid[start.m_nXPos][start.m_nYPos].nGridValue == 3)
    {
        qDebug() << "...........Start not walkable.............." ;
        return false;
    }
    if((goal.m_nXPos < 0) || (goal.m_nYPos < 0) || (goal.m_nXPos == nWd) || (goal.m_nYPos == nHt))
    {
        qDebug() << "...........Out of bound goal.............." ;
        return false;
    }
    if(grid[goal.m_nXPos][start.m_nYPos].nGridValue == 3)
    {
        qDebug() << "...........Goal not walkable.............." ;
        return false;
    }

    priority_queue<CGridItem, vector<CGridItem>, greater<CGridItem>> pqOpenList;

    CGridItem   workItem = grid[start.m_nXPos][start.m_nYPos];
    grid[start.m_nXPos][start.m_nYPos].status = 1;
    pqOpenList.push(workItem);
    while (!pqOpenList.empty())
    {
        CGridItem current = pqOpenList.top();
        pqOpenList.pop();
        grid[current.m_pos.m_nXPos][current.m_pos.m_nYPos].status = -1;

        if (current.m_pos == goal) {
            current = grid[goal.m_nXPos][goal.m_nYPos];
            //grid[goal.m_nXPos][goal.m_nYPos].nGridValue = 3;
            do{
                qDebug() << "...........Path : " << current.m_pos.m_nXPos << ":" << current.m_pos.m_nYPos;
                grid[current.m_pos.m_nXPos][current.m_pos.m_nYPos].nGridValue = 12;
                current = grid[current.m_Parent.m_nXPos][current.m_Parent.m_nYPos];
            }while((current.m_pos.m_nXPos != start.m_nXPos) || (current.m_pos.m_nYPos != start.m_nYPos) );
            //grid[current.m_pos.m_nXPos][current.m_pos.m_nYPos].nGridValue = 3;
            qDebug() << "...........Start : " << current.m_pos.m_nXPos << ":" << current.m_pos.m_nYPos;
            grid[start.m_nXPos][start.m_nYPos].nGridValue = 11;
            grid[goal.m_nXPos][goal.m_nYPos].nGridValue = 13;
            return true;
        }

        // Explore neighbors
        for (int i = 0; i < 4; ++i)
        {
            int newX = current.m_pos.m_nXPos + directionX[i];
            int newY = current.m_pos.m_nYPos + directionY[i];

            // Check if the neighbor is within the grid boundaries
            if (newX >= 0 && newX < nWd && newY >= 0 && newY < nHt)
            {
                // Check if the neighbor is walkable and not in the closed list
                if (grid[newX][newY].nGridValue != 3 && (grid[newX][newY].status != -1))
                {
                    CGridItem   neighbor = grid[newX][newY];
                    int newG = current.distFromStart + 10;

                    // Check if the neighbor is not in the open list or has a lower g value
                    if (newG < neighbor.distFromStart || (grid[newX][newY].status != 1))
                    {
                        neighbor.distFromStart = newG;
                        neighbor.distEstToGoal = abs(newX - goal.m_nXPos) + abs(newY - goal.m_nYPos);
                        neighbor.distTotal = neighbor.distFromStart + neighbor.distEstToGoal;
                        //grid[newX][newY] = current; // Update the parent of the neighbor
                        grid[newX][newY].m_Parent = current.m_pos;
                        grid[newX][newY].status = 1;
                        pqOpenList.push(neighbor); // Add the neighbor to the open list
                    }
                }
            }
        }
    }
    return false;
}

//RandomDataGenerator() is used for generating random data for testing
// As proposed design this can be separated behind data generator interface - that will be used internally by the Model
vector<vector<int>> RandomDataGenerator()
{
    vector<vector<int>> newData;
    vector<int> newRow;
    //Use Randomize testing
    nWd = constMAXSIZE;
    nHt = constMAXSIZE;
    CPosition start(abs((int)QRandomGenerator::global()->generate())%nWd, abs((int)QRandomGenerator::global()->generate())%nHt);
    CPosition goal(abs((int)QRandomGenerator::global()->generate())%nWd, abs((int)QRandomGenerator::global()->generate())%nHt);
    //CPosition start(1,1);
    //CPosition goal(3,17);

    vector<vector<CGridItem>> grid;
    vector<CGridItem> testGridItems1;
    for(int i = 0; i < nHt; i++)
    {
        for(int j = 0; j < nWd; j++)
        {
            //Use Randomize testing
            int vVal = (abs((int)QRandomGenerator::global()->generate())%8<7?-1:3);
            testGridItems1.push_back(CGridItem(i,j, vVal));
        }
        grid.push_back(testGridItems1);
        testGridItems1.clear();
    }
    grid[start.m_nXPos][start.m_nYPos].nGridValue = 0;
    grid[goal.m_nXPos][goal.m_nYPos].nGridValue = 8;

    bool bFound = FindPath(grid, nWd, nHt, start, goal);
    qDebug() << "======================================" << bFound ;
    for(int i = 0; i < nHt; i++)
    {
        for(int j = 0; j < nWd; j++)
        {
            newRow.push_back(grid[i][j].nGridValue);
        }
        newData.push_back(newRow);
        newRow.clear();
    }
    return newData;
}

//loadData() is used for loading data from Riskylab compatible json files
// As proposed design this can be separated behind data generator interface - that will be used internally by the Model
// This can also be updated for generic json file loader, so it wont be restricted to Riskylab format
vector<vector<int>> loadData(QString strFileToOpen)
{
    vector<int>    vecJsonData;
    bool    bJsonDataAcquired = false;
    bool    bPaddingRequired = false;
    vector<vector<int>> newData;
    vector<int> newRow;

    // JSON Parseer
    //QFile file("/Users/user/Workarea/TestFldr/QQMLTest1/map.json");
    QFile file(strFileToOpen);
    QString errMsg;
    QFileDevice::FileError err = QFileDevice::NoError;
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        errMsg = file.errorString();
        qDebug() << "Could not open the file." << errMsg;
        return newData;
    }
    QByteArray jsonData = file.readAll();
    file.close();

    QJsonParseError error;
    QJsonDocument document = QJsonDocument::fromJson(jsonData, &error);
    if (error.error != QJsonParseError::NoError) {
        qDebug() << "Error parsing JSON:" << error.errorString();
        return newData;
    }
    if (document.isObject()) {
        QJsonObject obj = document.object();
        QString key;
        QJsonValue value;
        foreach (key, obj.keys()) {
            if(bJsonDataAcquired)   break;
            value = obj.value(key);
            if(key == "canvas")
                continue;
            else if(key == "layers")
                qDebug() << key;
            else if(key == "tilesets")
                continue;

            if (value.isString()) {
                continue;
            } else if (value.isDouble()) {
                continue;
            } else if (value.isBool()) {
                continue;
            } else if (value.isObject()) {
                continue;
            } else if (value.isArray()) {
                QJsonArray jArr(value.toArray());
                for (int i = 0; i < jArr.size(); ++i)
                {
                    if (jArr[i].isObject()) {
                        QJsonObject obj = jArr[i].toObject();
                        //if(obj.keys().count() == 3)
                        {
                            // Iterate through the keys and values
                            for (auto it = obj.constBegin(); it != obj.constEnd(); ++it) {
                                QString key = it.key();
                                if(key == "data")
                                {
                                    QJsonValue value = it.value();
                                    if (value.isArray()) {
                                        QJsonArray grades = value.toArray();
                                        //Check input data size for MAX
                                        if(grades.count() > (constMAXSIZE * constMAXSIZE)) break;
                                        for (const auto& grade : grades) {
                                            vecJsonData.push_back(grade.toInt());//qDebug() << grade.toInt();
                                        }
                                        bJsonDataAcquired = true;
                                    }
                                }
                            }
                        }
                    }
                }

            } else {
                qDebug() << key << ": null";
            }
        }
    }
    // EO JSON Parseer

    //Clean JSON Data & adjust width, height
    // width & height are adjusted to the nearest square format
    bPaddingRequired = ( (vecJsonData.size()%2)?true:false );
    nWd = round( sqrt(vecJsonData.size()));
    nHt = ceil( sqrt(vecJsonData.size()));

    //Data cleaning - As noticed data in the Riskylab json can be invalid (out of required data definations)
    // This cleaning takes care of handling that data - invalid data cell is converted to walkable cell
    bool bFoundStart = false;
    bool bFoundGoal = false;
    for (auto it = vecJsonData.begin(); it != vecJsonData.end(); ++it) {
        switch(*it)
        {
        case 0: // start
            if(bFoundStart) *it = -1; else bFoundStart = true;
            break;
        case 8: //goal
            if(bFoundGoal) *it = -1; else bFoundGoal = true;
            break;
        case 3: //not walkable
            break;
        default: //walkable
            *it = -1;
            break;
        }
    }
    //In case padding data is required, all data is added as walkable cell items
    // Pad data
    int nPad = (nWd * nHt) - vecJsonData.size();
    for (int nIdx = 0; nIdx < nPad; ++nIdx) {
        vecJsonData.push_back(-1);
    }
    //Default start & goal to invalid, later it will be populated from json data
    CPosition start(-1,-1);
    CPosition goal(-1,-1);

    vector<vector<CGridItem>> grid;
    vector<CGridItem> testGridItems1;
    for(int i = 0; i < nHt; i++)
    {
        for(int j = 0; j < nWd; j++)
        {
            if(vecJsonData[i * nWd + j] == 0)
            {
                start.m_nXPos = i; start.m_nYPos = j;
            }
            else if(vecJsonData[i * nWd + j] == 8)
            {
                goal.m_nXPos = i; goal.m_nYPos = j;
            }
            testGridItems1.push_back(CGridItem(i,j, vecJsonData[i * nWd + j]));
        }
        grid.push_back(testGridItems1);
        testGridItems1.clear();
    }
    // A* search for path finding
    bool bFound = FindPath(grid, nWd, nHt, start, goal);
    qDebug() << "======================================" << bFound ;

    //Return data preparation - ideally this will be not required with proper design
    for(int i = 0; i < nHt; i++)
    {
        for(int j = 0; j < nWd; j++)
        {
            newRow.push_back(grid[i][j].nGridValue);
        }
        newData.push_back(newRow);
        newRow.clear();
    }
    return newData;
}

//Qt Model to handle data & also interfacing with the Qt view for showing
class MyTableModel : public QAbstractTableModel {
public:
    MyTableModel(QObject *parent = nullptr) : QAbstractTableModel(parent) {}

    int rowCount(const QModelIndex &parent = QModelIndex()) const override {
        return mydata.size();
    }

    int columnCount(const QModelIndex &parent = QModelIndex()) const override {
        return mydata.empty() ? 0 : mydata[0].size();
    }

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override {
        if (!index.isValid())
            return QVariant();

        if (role == Qt::DisplayRole) {
            return mydata[index.row()][index.column()];
        }
        if (role == Qt::BackgroundRole) {
            if (mydata [index.row()][index.column()] == 3) // Non walkable cell in Red
                return QBrush(Qt::red);
            if (mydata [index.row()][index.column()] == 11)//Start of the path
                return QBrush(Qt::green);
            if (mydata [index.row()][index.column()] == 12)//Path
                return QBrush(Qt::darkGreen);
            if (mydata [index.row()][index.column()] == 13)//Destination
                return QBrush(Qt::blue);
            else
                return QBrush(Qt::lightGray); // Default cell color
        }
        return QVariant();
    }
    //setDataVector() is used for setting Model data
    void setDataVector(const vector<vector<int>>& newData) {
        mydata = move(newData);
    }
    //When data source is json, following method is called from vie passing file name
    // file is later parsed for data from behind the scene data provider & path finding is performed (ideally they will be separated in a system)
    void setJsonDataSource(QString strFileName){
        mydata = loadData(strFileName);
    }
    //Function to generate random test data - invoked from a button click on view
    void setRandomTestDataGenerator(){
        mydata = RandomDataGenerator();
    }

private:
    vector<vector<int>> mydata;
};

// Main program
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget wMainWidget;

    MyTableModel *modelDataSource = new MyTableModel();
    QTableView *viewTable = new QTableView();
    viewTable->setModel(modelDataSource);

    QString fileName;
    QPushButton btnSelectJSONfile("Select Riskylab File");
    QObject::connect(&btnSelectJSONfile, &QPushButton::clicked, [&]() {
        fileName = QFileDialog::getOpenFileName(
            nullptr,
            "Select a file",
            "",
            "All Files (*.json)"
            );
        // ideally this will be shown to user on app - for the future!
        if (!fileName.isEmpty()) {
            qDebug() << "Selected file:" << fileName;
        } else {
            qDebug() << "No file selected";
        }
        modelDataSource->setJsonDataSource(fileName);
        modelDataSource->layoutChanged();
    });

    QPushButton btnRandomTestGenerator("Click for Random test");
    QObject::connect(&btnRandomTestGenerator, &QPushButton::clicked, [&]() {
        modelDataSource->setRandomTestDataGenerator();
        modelDataSource->layoutChanged();
    });

    // Set layout for the main widget
    QHBoxLayout *layout = new QHBoxLayout;
    QVBoxLayout *vLayout = new QVBoxLayout;
    vLayout->addWidget(&btnSelectJSONfile);
    vLayout->addWidget(&btnRandomTestGenerator);
    layout->addLayout(vLayout);
    layout->addWidget(viewTable);
    wMainWidget.setLayout(layout);
    wMainWidget.show();
    return a.exec();
}
