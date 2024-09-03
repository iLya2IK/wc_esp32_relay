package main

import (
	"container/list"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"sync"
	"sync/atomic"
	"time"

	wc "github.com/ilya2ik/wcwebcamclient_go"
)

const MAX_DAY_TEMPERATURE = -10.0
const MIN_DAY_TEMPERATURE = -20.0
const WORLD_TIME_SCALE = 5.0

const T_RELAY_CONFIG_FILE = "trelay_config.json"
const JSON_RPC_T = "t"
const JSON_RPC_ID = "id"
const JSON_RPC_TRG_T = "trgt"
const JSON_RPC_EXT_POW = "extp"
const JSON_RPC_HYST = "hyst"
const JSON_RPC_HEAT = "heat"
const JSON_RPC_VALUE = "value"
const JSON_RPC_VALUES = "values"
const JSON_RPC_START_BRD = "startbrd"
const JSON_RPC_STOP_BRD = "stopbrd"
const JSON_RPC_IS_BRD = "isbrd"
const JSON_RPC_STATUS = "status"
const JSON_RPC_GET_T = "gett"
const JSON_RPC_MODE = "mode"
const JSON_RPC_CUR_MODE = "curmode"
const JSON_RPC_RUN_MODE = "runmode"
const JSON_RPC_GET_MODE = "getmode"
const JSON_RPC_SET_MODE = "setmode"
const JSON_RPC_SET_HYST = "sethyst"
const JSON_RPC_GET_HYST = "gethyst"
const JSON_RPC_GET_EXT_POW = "getextp"
const JSON_RPC_LST_MODES = "lstmodes"

func check(e error) {
	if e != nil {
		log.Panic(e)
	}
}

type WCServer struct {
	Host      string `json:"host"`
	Port      int    `json:"port"`
	VerifyTLS bool   `json:"verifyTLS"`
	Proxy     string `json:"proxy"`
}

type EmulatorConfig struct {
	Debug        bool      `json:"debug"`
	DeviceName   string    `json:"device_name"`
	AccountLogin string    `json:"account_login"`
	AccountPwd   string    `json:"account_pwd"`
	Modes        []float64 `json:"modes"`
	CurMode      int       `json:"cur_mode"`
	IsBroad      bool      `json:"is_broad"`
	Hyst         float64   `json:"hyst"`
	WCSrv        WCServer  `json:"wcserver"`
}

type TRelayConfig struct {
	Modes   []float64 `json:"modes"`
	CurMode int       `json:"cur_mode"`
	IsBroad bool      `json:"is_broad"`
	Hyst    float64   `json:"hyst"`
}

const MAX_T_MODES int = 3

type RelayStatus struct {
	world_mux      sync.RWMutex
	world_ExTemp   float64
	world_Temp     float64
	world_HeatTemp float64
	world_ExtPower bool
	world_CurTime  float64

	modeMeasureT  atomic.Bool
	modeHasT      atomic.Bool
	modeNeedBroad atomic.Bool

	isBroadcast   atomic.Bool
	hasExtPower   atomic.Bool
	mHasExtPower  atomic.Bool
	isHeating     atomic.Bool
	mIsHeating    atomic.Bool
	varsChanged   atomic.Bool
	modes         [MAX_T_MODES]float64
	curMode       int
	temperature   float64
	hyst          float64
	broadcastTime atomic.Int32

	lst_brd_status_ext_pow  bool
	lst_brd_status_heating  bool
	lst_brd_status_t        float64
	lst_brd_status_target_t float64

	startedAt time.Time

	mux sync.RWMutex
}

type OutMessagesListThreadSafe struct {
	mux sync.Mutex

	value *list.List
}

func (c *OutMessagesListThreadSafe) PushBack(msg *wc.OutMessageStruct) {
	c.mux.Lock()
	defer c.mux.Unlock()

	c.value.PushBack(msg)
}

func (c *OutMessagesListThreadSafe) NotEmpty() bool {
	c.mux.Lock()
	defer c.mux.Unlock()

	return c.value.Len() > 0
}

func (c *OutMessagesListThreadSafe) Pop() *wc.OutMessageStruct {
	c.mux.Lock()
	defer c.mux.Unlock()

	el := c.value.Front()
	if el != nil {
		return c.value.Remove(el).(*wc.OutMessageStruct)
	} else {
		return nil
	}
}

func (c *OutMessagesListThreadSafe) Clear() {
	c.mux.Lock()
	defer c.mux.Unlock()

	c.value = list.New()
}

var client *wc.WCClient
var outgoing_msgs *OutMessagesListThreadSafe = &OutMessagesListThreadSafe{mux: sync.Mutex{}, value: new(list.List)}
var status *RelayStatus
var cur_cfg *TRelayConfig
var lst_tv time.Time

/* status world Block */

func (status *RelayStatus) SetWorldTemperature(value float64) {
	status.world_mux.Lock()
	defer status.world_mux.Unlock()

	status.world_Temp = value
}

func (status *RelayStatus) SetWorldHasExtPower(value bool) {
	status.world_mux.Lock()
	defer status.world_mux.Unlock()

	status.world_ExtPower = value
}

func (status *RelayStatus) WorldTemperature() float64 {
	status.world_mux.RLock()
	defer status.world_mux.RUnlock()

	return status.world_Temp
}

func (status *RelayStatus) SetWorldOutdoorTemperature(value float64) {
	status.world_mux.Lock()
	defer status.world_mux.Unlock()

	status.world_ExTemp = value
}

func (status *RelayStatus) WorldOutdoorTemperature() float64 {
	status.world_mux.RLock()
	defer status.world_mux.RUnlock()

	return status.world_ExTemp
}

func (status *RelayStatus) SetWorldHeatTemperature(value float64) {
	status.world_mux.Lock()
	defer status.world_mux.Unlock()

	status.world_HeatTemp = value
}

func (status *RelayStatus) WorldHeatTemperature() float64 {
	status.world_mux.RLock()
	defer status.world_mux.RUnlock()

	return status.world_HeatTemp
}

func (status *RelayStatus) WorldHasExtPower() bool {
	status.world_mux.Lock()
	defer status.world_mux.Unlock()

	return status.world_ExtPower
}

func (status *RelayStatus) AddCurrentTime(sc float64) float64 {
	status.world_mux.Lock()
	defer status.world_mux.Unlock()

	status.world_CurTime += sc
	return status.world_CurTime
}

/* status */

func (status *RelayStatus) SetModeMeasureT(value bool) {
	status.modeMeasureT.Store(value)
}

func (status *RelayStatus) SetModeHasT(value bool) {
	status.modeHasT.Store(value)
}

func (status *RelayStatus) SetModeNeedBroadcast(value bool) {
	status.modeNeedBroad.Store(value)
}

func (status *RelayStatus) ModeMeasureT() bool {
	return status.modeMeasureT.Load()
}

func (status *RelayStatus) ModeHasT() bool {
	return status.modeHasT.Load()
}

func (status *RelayStatus) ModeNeedBroadcast() bool {
	return status.modeNeedBroad.Load()
}

func (status *RelayStatus) setVarsChanged() {
	status.varsChanged.Store(true)
}

func (status *RelayStatus) resetVarsChanged() {
	status.varsChanged.Store(false)
}

func (status *RelayStatus) isVarsChanged() bool {
	return status.varsChanged.Load()
}

func (status *RelayStatus) Broadcast() bool {
	return status.isBroadcast.Load()
}

func (status *RelayStatus) SetBroadcast(value bool) {
	if value != status.Broadcast() {
		status.isBroadcast.Store(value)
		status.setVarsChanged()
	}
}

func (status *RelayStatus) BroadcastTime() int {
	return int(status.broadcastTime.Load())
}

func (status *RelayStatus) SetBroadcastTime(value int) {
	status.broadcastTime.Store(int32(value))
}

func (status *RelayStatus) TModeValue(id int) float64 {
	status.mux.RLocker().Lock()
	defer status.mux.RLocker().Unlock()

	return status.modes[id]
}

func (status *RelayStatus) SetTMode(id int, value float64) {
	status.mux.Lock()
	defer status.mux.Unlock()

	if value < 0 {
		value = 0.0
	}
	if value > 50.0 {
		value = 50.0
	}

	if status.modes[id] != value {
		status.modes[id] = value
		status.setVarsChanged()
	}
}

func (status *RelayStatus) CurTrgModeId() int {
	status.mux.RLocker().Lock()
	defer status.mux.RLocker().Unlock()

	return status.curMode
}

func (status *RelayStatus) CurTrgModeValue() float64 {
	status.mux.RLocker().Lock()
	defer status.mux.RLocker().Unlock()

	return status.modes[status.curMode]
}

func (status *RelayStatus) SetCurTrgModeId(id int) {
	status.mux.Lock()
	defer status.mux.Unlock()

	if status.curMode != id {
		status.curMode = id
		status.setVarsChanged()
	}
}

func (status *RelayStatus) Temperature() float64 {
	status.mux.RLocker().Lock()
	defer status.mux.RLocker().Unlock()

	return status.temperature
}

func (status *RelayStatus) SetTemperature(value float64) {
	status.mux.Lock()
	defer status.mux.Unlock()

	status.temperature = value
}

func (status *RelayStatus) Hyst() float64 {
	status.mux.RLocker().Lock()
	defer status.mux.RLocker().Unlock()

	return status.hyst
}

func (status *RelayStatus) SetHyst(value float64) {
	status.mux.Lock()
	defer status.mux.Unlock()

	if value < 0.125 {
		value = 0.125
	}
	if value > 2.5 {
		value = 2.5
	}

	if status.hyst != value {
		status.hyst = value
		status.setVarsChanged()
	}
}

func (status *RelayStatus) ExtPower() bool {
	return status.hasExtPower.Load()
}

func (status *RelayStatus) SetExtPower(value bool) {
	status.mHasExtPower.Store(value)
}

func (status *RelayStatus) SyncExtPower() bool {
	var measured = status.mHasExtPower.Load()
	var res = measured != status.hasExtPower.Load()
	if res {
		status.hasExtPower.Store(measured)
	}
	return res
}

func (status *RelayStatus) Heating() bool {
	return status.isHeating.Load()
}

func (status *RelayStatus) SetHeating(value bool) {
	status.mIsHeating.Store(value)
}

func (status *RelayStatus) SyncHeating() bool {
	var supplied = status.mIsHeating.Load()
	var res = supplied != status.isHeating.Load()
	if res {
		status.isHeating.Store(supplied)
	}
	return res
}

func AuthSuccess(tsk wc.ITask) {
	fmt.Println("SID ", tsk.GetClient().GetSID())
}

func OnLog(client *wc.WCClient, str string) {
	fmt.Println(str)
}

func OnClientStateChange(c *wc.WCClient, st wc.ClientStatus) {
	switch st {
	case wc.StateConnected:
		fmt.Printf("Client fully connected to server with sid %s\n", c.GetSID())
	case wc.StateConnectedWrongSID:
		fmt.Printf("Client has no SID\n")
	case wc.StateDisconnected:
		fmt.Printf("Client disconnected\n")
	}
}

type TRParam = struct {
	T       *float64   `json:"t,omitempty"`
	ExtP    *bool      `json:"extp,omitempty"`
	Heat    *bool      `json:"heat,omitempty"`
	IsBrd   *bool      `json:"isbrd,omitempty"`
	Mode    *int       `json:"mode,omitempty"`
	Hyst    *float64   `json:"hyst,omitempty"`
	TargetT *float64   `json:"trgt,omitempty"`
	ModeId  *int       `json:"id,omitempty"`
	Value   *float64   `json:"value,omitempty"`
	Values  []*float64 `json:"values,omitempty"`
}

type TRMessage struct {
	Device *string  `json:"device,omitempty"`
	Msg    string   `json:"msg"`
	Params *TRParam `json:"params,omitempty"`
}

func sendModeId(msg map[string]any, modeid int, value float64) {
	msg[JSON_RPC_ID] = modeid
	msg[JSON_RPC_VALUE] = value
}

func sendIsBroad(src_s string) {
	msg := &wc.OutMessageStruct{
		Target: src_s,
		Msg:    JSON_RPC_IS_BRD,
		Params: make(map[string]any),
	}
	msg.Params[JSON_RPC_IS_BRD] = status.Broadcast()
	outgoing_msgs.PushBack(msg)
}

type ModeOp int

const (
	OpGet ModeOp = iota
	OpSet
	OpRun
)

func modeOp(op ModeOp, msg map[string]any, incoming *TRParam) {
	if incoming != nil && incoming.ModeId != nil {
		var id_var int = *incoming.ModeId
		if id_var < MAX_T_MODES {
			switch op {
			case OpGet:
				{
					v := status.TModeValue(id_var)
					sendModeId(msg, id_var, v)
				}
			case OpSet:
				{
					if incoming.Value == nil {
						break
					}
					var new_value = *incoming.Value
					status.SetTMode(id_var, new_value)
					v := status.TModeValue(id_var)
					sendModeId(msg, id_var, v)
				}
			case OpRun:
				{
					status.SetCurTrgModeId(id_var)
				}
			}
		}
	}
}

type updateStatus struct {
	Temperature bool
	Target      bool
	ModeId      bool
	Hyst        bool
	ExtPower    bool
	Heating     bool
	Modes       bool
	IsBroad     bool
}

var fullStatusSend = updateStatus{true, true, true, true, true, true, false, false}
var clrStatusSend = updateStatus{false, false, false, false, false, false, false, false}

func sendStatus(src_s string, mask *updateStatus) {
	omsg := &wc.OutMessageStruct{
		Target: src_s,
		Msg:    JSON_RPC_STATUS,
		Params: make(map[string]any),
	}

	if (mask.Temperature || mask.Heating || mask.ExtPower) && status.ModeHasT() {
		omsg.Params[JSON_RPC_T] = status.Temperature()
	}
	if (mask.Target) || (mask.ModeId) {
		omsg.Params[JSON_RPC_TRG_T] = status.CurTrgModeValue()
		omsg.Params[JSON_RPC_ID] = status.CurTrgModeId()
	}
	if mask.Hyst {
		omsg.Params[JSON_RPC_HYST] = status.Hyst()
	}
	if mask.ExtPower {
		omsg.Params[JSON_RPC_EXT_POW] = status.ExtPower()
	}
	if mask.Heating {
		omsg.Params[JSON_RPC_HEAT] = status.Heating()
	}
	outgoing_msgs.PushBack(omsg)
}

type targetDevice struct {
	name   string
	status updateStatus
}

var status_targets []*targetDevice = make([]*targetDevice, 32)
var status_target_offset int = -1

func resetStatusTargetLocs() {
	for i := 0; i < len(status_targets); i++ {
		if status_targets[i] != nil {
			status_targets[i].status = clrStatusSend
		}
	}
}

func getTargetStatusLoc(aname string) *targetDevice {
	for i := 0; i < len(status_targets); i++ {
		if status_targets[i] != nil {
			if status_targets[i].name == aname {
				return status_targets[i]
			}
		}
	}
	status_target_offset++
	if status_target_offset >= len(status_targets) {
		status_targets = append(status_targets, nil)
	}
	if status_targets[status_target_offset] == nil {
		status_targets[status_target_offset] = &targetDevice{name: aname}
	}
	return status_targets[status_target_offset]
}

func consumeStatusTargetLocs() {
	for _, trg := range status_targets {
		if trg != nil {
			if trg.status.ExtPower ||
				trg.status.Temperature ||
				trg.status.Heating ||
				trg.status.Hyst ||
				trg.status.Target ||
				trg.status.ModeId {
				sendStatus(trg.name, &trg.status)
			}
			if trg.status.IsBroad {
				sendIsBroad(trg.name)
			}
			if trg.status.Modes {
				omsg := &wc.OutMessageStruct{
					Target: trg.name,
					Msg:    JSON_RPC_LST_MODES,
					Params: make(map[string]any),
				}
				arr := make([]*float64, MAX_T_MODES)
				for i := 0; i < MAX_T_MODES; i++ {
					arr[i] = new(float64)
					*arr[i] = status.TModeValue(i)
				}
				omsg.Params[JSON_RPC_VALUES] = arr
				outgoing_msgs.PushBack(omsg)
			}
		}
	}
}

func refreshExtPow() {
	status.SetExtPower(status.WorldHasExtPower())
}

func sendBroadcast() {
	if status.ModeNeedBroadcast() {
		status.SetModeNeedBroadcast(false)

		if status.Broadcast() {
			tv := time.Now().UTC()
			duration := tv.Sub(status.startedAt)
			status.SetBroadcastTime(int(duration.Seconds()))

			status.lst_brd_status_ext_pow = status.ExtPower()
			status.lst_brd_status_heating = status.Heating()
			status.lst_brd_status_t = status.Temperature()
			status.lst_brd_status_target_t = status.modes[status.curMode]

			sendStatus("", &fullStatusSend)
		}
	}
}

func measureExtPower() {
	if status.SyncExtPower() {
		if client.GetClientStatus() == wc.StateConnected {
			sendStatus("", &updateStatus{Temperature: true, ExtPower: true})
		}
	}
}

func measureHeating() {
	if status.SyncHeating() {
		if client.GetClientStatus() == wc.StateConnected {
			sendStatus("", &updateStatus{Temperature: true, Heating: true})
		}
	}
}

func measureTemperature() {
	status.SetModeHasT(true)
	status.SetModeMeasureT(false)
	status.SetTemperature(status.WorldTemperature())
}

func updateRelay() {
	if status.ExtPower() {
		if status.ModeHasT() {
			// Get the current temperature value from the state registers
			temp := status.Temperature()
			// Get the target temperature value from the state registers
			mtemp := status.CurTrgModeValue()
			// Get the hysteresis value from the state registers
			htemp := status.Hyst()
			// Recalculate the difference between the measured and target temperatures
			dt := temp - mtemp

			if dt >= htemp {
				// send the SWITCH_OUT_OFF status to the control pin
				// gpio_set_level(OUT_SWITCH, SWITCH_OUT_OFF);
				// Change the status of the state machine
				status.SetHeating(false)
			} else if dt <= -htemp {
				// send the SWITCH_OUT_ON status to the control pin
				// gpio_set_level(OUT_SWITCH, SWITCH_OUT_ON);
				// Change the status of the state machine
				status.SetHeating(true)
			}
		}
	} else {
		// later
	}
}

func main() {
	lst_tv = time.Now().UTC()

	status = &RelayStatus{
		// real world status
		world_mux:      sync.RWMutex{},
		world_ExTemp:   -20.0, // outdoors temperature
		world_Temp:     20.0,  // room temperature
		world_HeatTemp: 20.0,  // heater temperature
		world_ExtPower: true,  // has ext power
		world_CurTime:  0.0,   // current time
		// thermo relay status
		modeMeasureT:            atomic.Bool{},
		modeHasT:                atomic.Bool{},
		modeNeedBroad:           atomic.Bool{},
		isBroadcast:             atomic.Bool{},
		hasExtPower:             atomic.Bool{},
		mHasExtPower:            atomic.Bool{},
		isHeating:               atomic.Bool{},
		mIsHeating:              atomic.Bool{},
		varsChanged:             atomic.Bool{},
		modes:                   [MAX_T_MODES]float64{5.0, 20.0, 23.0},
		curMode:                 0,
		temperature:             0.0,
		hyst:                    1.0,
		lst_brd_status_ext_pow:  false,
		lst_brd_status_heating:  false,
		lst_brd_status_t:        0.0,
		lst_brd_status_target_t: 0.0,
		broadcastTime:           atomic.Int32{},
		startedAt:               time.Now().UTC(),
		mux:                     sync.RWMutex{},
	}

	cfgFile, err := os.Open("config.json")
	check(err)
	byteValue, err := io.ReadAll(cfgFile)
	check(err)
	var wce_cfg EmulatorConfig
	err = json.Unmarshal(byteValue, &wce_cfg)
	check(err)
	cfgFile.Close()

	cfgStateFile, err := os.Open(T_RELAY_CONFIG_FILE)
	if err == nil {
		cur_cfg = &TRelayConfig{Modes: make([]float64, MAX_T_MODES)}

		byteValue, err := io.ReadAll(cfgStateFile)
		check(err)
		err = json.Unmarshal(byteValue, cur_cfg)
		check(err)
		cfgStateFile.Close()

		status.SetCurTrgModeId(cur_cfg.CurMode)
		status.SetBroadcast(cur_cfg.IsBroad)
		status.SetHyst(cur_cfg.Hyst)

		for i, v := range cur_cfg.Modes {
			if i < MAX_T_MODES {
				status.modes[i] = v
			}
		}
	}

	cfg := wc.ClientCfgNew()
	if len(wce_cfg.WCSrv.Host) > 0 {
		cfg.SetHostURL(wce_cfg.WCSrv.Host)
	} else {
		cfg.SetHostURL("https://127.0.0.1")
	}
	if wce_cfg.WCSrv.Port > 0 {
		cfg.SetPort(wce_cfg.WCSrv.Port)
	}
	if len(wce_cfg.WCSrv.Proxy) > 0 {
		cfg.SetProxy(wce_cfg.WCSrv.Proxy)
	}
	cfg.SetVerifyTLS(wce_cfg.WCSrv.VerifyTLS)

	var new_cfg *wc.WCClientConfig = wc.ClientCfgNew()
	new_cfg.AssignFrom(cfg)
	new_cfg.SetDevice(wce_cfg.DeviceName)

	client, err = wc.ClientNew(new_cfg)
	check(err)
	client.SetNeedToSync(true)
	client.SetLstMsgStampToSyncPoint()
	client.SetOnAuthSuccess(AuthSuccess)
	client.SetOnAddLog(OnLog)
	client.SetOnConnected(OnClientStateChange)

	fmt.Println("Trying to start client")

	check(client.Start())

	fmt.Println("Client started")

	type fire struct {
		command   func(...any) error
		onsuccess any
		timeout   int64
		mask      []wc.ClientStatus
	}

	sheduler := make(chan *fire, 6)
	sheduler <- &fire{ // consume incoming messages
		command: client.UpdateMsgs,
		onsuccess: func(tsk wc.ITask, res []map[string]any) {
			resetStatusTargetLocs()
			for _, m := range res {
				jsonString, _ := json.Marshal(m)
				var msg TRMessage
				if err := json.Unmarshal(jsonString, &msg); err != nil {
					check(err)
				}

				if msg.Device == nil || msg.Msg == "" || *msg.Device == wce_cfg.DeviceName {
					continue
				}

				var target *targetDevice = getTargetStatusLoc(*msg.Device)

				if target == nil {
					continue
				}

				switch msg.Msg {
				// broad
				case JSON_RPC_START_BRD:
					{
						status.SetBroadcast(true)
						target.status.IsBroad = true
					}
				case JSON_RPC_STOP_BRD:
					{
						status.SetBroadcast(false)
						target.status.IsBroad = true
					}
				case JSON_RPC_IS_BRD:
					{
						target.status.IsBroad = true
					}
				// getters for status
				case JSON_RPC_STATUS:
					{
						target.status = fullStatusSend
					}
				case JSON_RPC_GET_T:
					{
						target.status.Temperature = true
					}
				case JSON_RPC_CUR_MODE:
					{
						target.status.ModeId = true
					}
				case JSON_RPC_GET_HYST:
					{
						target.status.Hyst = true
					}
				case JSON_RPC_GET_EXT_POW:
					{
						target.status.ExtPower = true
					}
				// setters for status
				case JSON_RPC_RUN_MODE:
					{
						modeOp(OpRun, nil, msg.Params)
						target.status.ModeId = true
					}
				case JSON_RPC_SET_HYST:
					{
						hyst_value := msg.Params.Hyst
						if hyst_value != nil {
							status.SetHyst(*hyst_value)
						}
						target.status.Hyst = true
					}
				// getters for modes
				case JSON_RPC_LST_MODES:
					{
						target.status.Modes = true
					}
				case JSON_RPC_GET_MODE:
					{
						omsg := &wc.OutMessageStruct{Target: target.name, Msg: JSON_RPC_MODE, Params: make(map[string]any)}
						modeOp(OpGet, omsg.Params, msg.Params)
						outgoing_msgs.PushBack(omsg)
					}
				// setters for modes
				case JSON_RPC_SET_MODE:
					{
						omsg := &wc.OutMessageStruct{Target: target.name, Msg: JSON_RPC_MODE, Params: make(map[string]any)}
						modeOp(OpSet, omsg.Params, msg.Params)
						outgoing_msgs.PushBack(omsg)
					}
				}
			}
			consumeStatusTargetLocs()
		},
		timeout: 4000,
		mask:    []wc.ClientStatus{wc.StateConnected},
	}
	sheduler <- &fire{ // sending outgoing messages
		command: func(...any) error {
			var sended = 0
			for sended < 16 {
				if outgoing_msgs.NotEmpty() {
					check(client.SendMsgs(outgoing_msgs.Pop()))
					sended++
				} else {
					break
				}
			}
			return nil
		},
		onsuccess: func(tsk wc.ITask, res []map[string]any) {
			for _, v := range res {
				fmt.Println(v)
			}
		},
		timeout: 1000,
		mask:    []wc.ClientStatus{wc.StateConnected},
	}
	sheduler <- &fire{ // broadcast
		command: func(...any) error {
			refreshExtPow()
			if client.GetClientStatus() == wc.StateConnected {
				var need_to_send bool = false
				if status.Broadcast() {
					tv := time.Now().UTC()
					duration := int(tv.Sub(status.startedAt).Seconds())
					if duration < status.BroadcastTime() {
						need_to_send = true
					} else if (duration - status.BroadcastTime()) >= 3000 { // every 5 minutes
						need_to_send = true
					} else if ((duration - status.BroadcastTime()) >= 30) && // every 30 seconds
						(((math.Abs(status.lst_brd_status_t - status.Temperature())) >= 0.25) ||
							(status.lst_brd_status_ext_pow != status.ExtPower()) ||
							(status.lst_brd_status_heating != status.Heating()) ||
							(status.lst_brd_status_target_t != status.modes[status.curMode])) {
						need_to_send = true
					}
				}
				if need_to_send {
					status.SetModeNeedBroadcast(true)
				}
			}
			return nil
		},
		timeout: 15000,
		mask: []wc.ClientStatus{
			wc.StateConnected,
		},
	}
	sheduler <- &fire{ // measure temperature
		command: func(...any) error {
			measureTemperature()
			return nil
		},
		timeout: 10000,
		mask: []wc.ClientStatus{
			wc.StateConnected,
		},
	}
	sheduler <- &fire{ // main loop
		command: func(...any) error {
			tv := time.Now().UTC()
			dur := tv.Sub(lst_tv)
			sc := dur.Seconds()
			lst_tv = tv

			cur_day_time := status.AddCurrentTime(sc*WORLD_TIME_SCALE) / 3600.0
			out_t := (1.0-math.Sin((cur_day_time+3.0)*math.Pi/12.0))*0.5*(MAX_DAY_TEMPERATURE-MIN_DAY_TEMPERATURE) + MIN_DAY_TEMPERATURE
			status.SetWorldOutdoorTemperature(out_t)

			var Q1, Q2, Q3 float64
			Q1 = 0.2 * (status.WorldTemperature() - status.WorldOutdoorTemperature())
			if status.Heating() {
				Q2 = 0.2 * (status.WorldHeatTemperature() - status.WorldTemperature())
				Q3 = 5.0 * (75.0 - status.WorldHeatTemperature())
			} else {
				Q3 = 0.15 * (status.WorldTemperature() - status.WorldHeatTemperature())
				Q2 = -Q3
			}
			Q := (Q2 - Q1) * sc * WORLD_TIME_SCALE * 0.001
			Q3 *= sc * WORLD_TIME_SCALE * 0.001

			status.SetWorldTemperature(Q + status.WorldTemperature())
			status.SetWorldHeatTemperature(Q3 + status.WorldHeatTemperature())

			if status.isVarsChanged() {
				// save vars
				cfgStateFile, err := os.Create(T_RELAY_CONFIG_FILE)
				if err == nil {
					cur_cfg = &TRelayConfig{}
					cur_cfg.CurMode = status.CurTrgModeId()
					cur_cfg.IsBroad = status.Broadcast()
					cur_cfg.Hyst = status.Hyst()

					status.mux.RLock()
					cur_cfg.Modes = make([]float64, MAX_T_MODES)
					copy(cur_cfg.Modes, status.modes[0:])
					status.mux.RUnlock()

					bytes, err := json.Marshal(cur_cfg)
					check(err)

					_, err = cfgStateFile.Write(bytes)
					check(err)

					cfgStateFile.Close()
				}
				status.resetVarsChanged()
			}
			measureExtPower()
			updateRelay()
			measureHeating()
			sendBroadcast()
			return nil
		},
		timeout: 250,
		mask: []wc.ClientStatus{
			wc.StateConnected,
			wc.StateConnectedWrongSID,
			wc.StateConnectedAuthorization,
		},
	}

	for loop := true; loop; {

		switch client.GetClientStatus() {
		case wc.StateConnectedWrongSID:
			{
				fmt.Println("Trying to authorize")
				check(client.Auth(wce_cfg.AccountLogin, wce_cfg.AccountPwd))
			}
		case wc.StateDisconnected:
			{
				loop = false
				break
			}
		default:
			{
				select {
				case v := <-sheduler:
					{
						go func(v *fire) {
							time.Sleep(time.Duration(v.timeout) * time.Millisecond)
							if client.IsClientStatusInRange(v.mask) {
								check(v.command(v.onsuccess))
							}
							sheduler <- v
						}(v)
					}
				default:
					{
						time.Sleep(250 * time.Millisecond)
					}
				}
			}
		}

	}

	close(sheduler)

	fmt.Println("Client finished")
}
