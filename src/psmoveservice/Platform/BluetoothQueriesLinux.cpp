// -- includes -----
#include "BluetoothQueries.h"
#include "ServerLog.h"
#include "ServerUtility.h"
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <assert.h>
#include <sstream>
#include <iomanip>
#include <vector>


//-- Queries -----
bool bluetooth_get_host_address(std::string &out_address)
{
    bool bSuccess = true;

    bdaddr_t btaddr;
    char pszaddr[18];

    // Assume only one bt adapter.
    int dev_id = hci_get_route(NULL);
    hci_devba(dev_id, &btaddr);
    ba2str(&btaddr, pszaddr);
    out_address.assign(pszaddr);

    return bSuccess;
}