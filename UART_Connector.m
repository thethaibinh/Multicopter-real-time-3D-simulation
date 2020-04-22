function varargout = UART_Connector(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @UART_Connector_OpeningFcn, ...
    'gui_OutputFcn',  @UART_Connector_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

function UART_Connector_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;
global ChannelOffset phi the psi x y z rc_in xlim ylim zlim
% global q0  q1  q2  q3 	% // quaternion elements representing the estimated orientation
% global exInt  eyInt  ezInt  %	// scaled integral error
% q0 = 0
% q1 = 0
% q2 = 0
% q3 = 0
% exInt = 0
% eyInt = 0
% ezInt = 0
rc_in = [0 0 0 0 0];             % RC input
% Axes limit
xlim = 10;
ylim = 10;
zlim = 15;
% Timers callback rate
update_rate = 30;
plot_rate = 20;
data_output = 10;
% Camera angle view
handles.AZval = get(handles.AZslider,'Value')*360; % Get azimuth view in degrees
handles.ELval = get(handles.ELslider,'Value')*90;

x = 0;
y = 0;
z = 0;
phi = 0;
the = 0;
psi = 0;
ChannelOffset = [1523 1521 1534 1518];
serialPorts = instrhwinfo('serial');
set(handles.popupmenu2, 'String', ...
    [{'Select a port'} ; serialPorts.SerialPorts ]);
set(handles.popupmenu2, 'Value', 1);
set(handles.popupmenu1, 'String', ...
    [{'Select baudrate'} ; {'4800'} ; {'9600'} ; {'19200'} ; {'38400'} ; {'57600'} ; {'115200'} ; {'230400'}]);
% Inititial axes
set(handles.ELslider,'Value',25/90);
set(handles.AZslider,'Value',234/360);
axes(handles.position_axes)
scatter3(x,y,z,'filled');
hold(handles.position_axes, 'on');
hold(handles.position_axes, 'on');
xlabel(handles.position_axes,'X (meters)');
ylabel(handles.position_axes,'Y (meters)');
zlabel(handles.position_axes,'Z (meters)');
grid minor;
axis equal;
set(handles.position_axes, 'ZLim', [0 zlim]);
set(handles.position_axes, 'XLim', [-xlim xlim]);
set(handles.position_axes, 'YLim', [-ylim ylim]);
view(handles.AZval,handles.ELval);
axes(handles.attitude_axes)
ax = gca;
hold(handles.attitude_axes, 'on');
xlabel(handles.attitude_axes,'X (centimeters)');
ylabel(handles.attitude_axes,'Y (centimeters)');
zlabel(handles.attitude_axes,'Z (centimeters)');
view(3);
grid(ax, 'on');
draw_uav(phi, the, psi, handles);
axis equal;
set(ax, 'ZLim', [-25 25]);
set(ax, 'XLim', [-37 37]);
set(ax, 'YLim', [-37 37]);
view(handles.AZval,handles.ELval);
handles.output = hObject;
% Inititial model
load_system('multi_model');
set_param('multi_model/Signal Receiver/ch1_input', 'Value', '1505');
set_param('multi_model/Signal Receiver/ch2_input', 'Value', '1505');
set_param('multi_model/Signal Receiver/ch3_input', 'Value', '1505');
set_param('multi_model/Signal Receiver/ch4_input', 'Value', '1484');
set_param('multi_model/Signal Receiver/ch5_input', 'Value', '0');
set_param('multi_model/Signal Receiver/ch1_offset', 'Value', sprintf('%d', ChannelOffset(1)));
set_param('multi_model/Signal Receiver/ch2_offset', 'Value', sprintf('%d', ChannelOffset(2)));
set_param('multi_model/Signal Receiver/ch3_offset', 'Value', sprintf('%d', ChannelOffset(3)));
set_param('multi_model/Signal Receiver/ch4_offset', 'Value', sprintf('%d', ChannelOffset(4)));

handles.input_data_timer = timer('ExecutionMode', 'fixedSpacing', 'Period', round(1000 / update_rate) / 1000, 'TimerFcn', {@update_data_input, handles});
handles.plot_timer = timer('ExecutionMode', 'fixedSpacing', 'Period', round(1000 / plot_rate) / 1000, 'TimerFcn', {@update_plot, handles});
handles.output_data_timer = timer('ExecutionMode', 'fixedSpacing', 'Period', round(1000 / data_output) / 1000, 'TimerFcn', {@update_data_output, handles});
g = imread('aselab-logo.png');

axes(handles.axes7)

imshow(g);
% Update handles structure
guidata(hObject, handles);

function varargout = UART_Connector_OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.output;

function connect_button_Callback(hObject, eventdata, handles)
if strcmp(get(hObject,'String'),'Connect RC')
    serPortn = get(handles.popupmenu2,'Value');
    BaudrateIndex = get(handles.popupmenu1, 'Value');
    if ((serPortn == 1) || (BaudrateIndex == 1))
        errordlg('Please select a COM port and baudrate')
    else
        set(hObject, 'String','Connecting RC');
        serList = get(handles.popupmenu2,'String');
        serPort = serList{serPortn};
        BaudrateList = get(handles.popupmenu1,'String');
        Baudrate = BaudrateList{BaudrateIndex};
        s = serial (serPort);
        handles.s = s;
        set(s,'BaudRate',str2double(Baudrate),'DataBits',8,'StopBits',1,...
            'Parity','none','FlowControl','none');  % set properties for serial
        s.BytesAvailableFcnMode = 'terminator'; % byte number or terminator
        set(s,'BytesAvailableFcn',{@update});
        try
            fopen(s);
            pause(3);
        catch e % Error notification
            errordlg(e.message);
            set(hobject,'String','Connect RC');
            set(hObject, 'BackgroundColor', 'g');
        end
        set(hObject, 'String','Disconnect RC');
        set(hObject, 'BackgroundColor', 'r');
    end
else
    if strcmp(get(handles.start_button,'String'),'Start')
        close_serial();
        set(hObject, 'String','Connect RC');
        set(hObject, 'BackgroundColor', 'g');
    else
        errordlg('Please stop simulation before disconnect RC')
    end
end
guidata(hObject, handles);

function update(obj, event, hObject, eventdata, handles)
global rc_in
buffer = fscanf(obj,'%d',[1,5]);% lay du lieu tu cam bien duoi dang mang so
if(length(buffer) == 5)
    rc_in = buffer;
    rc_in(5) = round((rc_in(5) - 1000)/500);
end

function update_plot(hObject, eventdata, handles)
global phi the psi x y z
%Perform attitude
draw_uav(phi, the, psi, handles);
%Perform XY position
draw_position(phi, the, psi, x, y, z, handles);

function update_data_input(hObject, eventdata, handles)
global ChannelOffset rc_in
% Update data
if (get(handles.pushbutton14,'Value') == 1)    
    set_param('multi_model/Signal Receiver/ch5_input', 'Value', sprintf('%d', 0));
else
    set_param('multi_model/Signal Receiver/ch5_input', 'Value', sprintf('%d', rc_in(5)));
    if (strcmp(get_param('multi_model', 'SimulationStatus'), 'running'))
        if (abs(rc_in(1) - ChannelOffset(1)) > 26)
            set_param('multi_model/Signal Receiver/ch1_input', 'Value', sprintf('%d', rc_in(1)));
        else
            set_param('multi_model/Signal Receiver/ch1_input', 'Value', sprintf('%d', ChannelOffset(1)));
        end

        if (abs(rc_in(2) - ChannelOffset(2)) > 26)
            set_param('multi_model/Signal Receiver/ch2_input', 'Value', sprintf('%d', rc_in(2)));
        else
            set_param('multi_model/Signal Receiver/ch2_input', 'Value', sprintf('%d', ChannelOffset(2)));
        end

        if (abs(rc_in(3) - ChannelOffset(3)) > 40)
            set_param('multi_model/Signal Receiver/ch3_input', 'Value', sprintf('%d', rc_in(3)));
        else
            set_param('multi_model/Signal Receiver/ch3_input', 'Value', sprintf('%d', ChannelOffset(3)));
        end

        if (abs(rc_in(4) - ChannelOffset(4)) > 26)
            set_param('multi_model/Signal Receiver/ch4_input', 'Value', sprintf('%d', rc_in(4)));
        else
            set_param('multi_model/Signal Receiver/ch4_input', 'Value', sprintf('%d', ChannelOffset(4)));
        end
    else
%         Check take off
        if (rc_in(3) >= ChannelOffset(3))
            set_param('multi_model','SimulationCommand','Start');
        end
    end
end



function update_data_output(hObject, eventdata, handles)
global phi the psi x y z xlim ylim zlim time
%Get data
if (strcmp(get_param('multi_model', 'SimulationStatus'), 'running'))
    simulation_data = get_param('multi_model/Visualization/simulation_data', 'RuntimeObject');
    x = -simulation_data.InputPort(4).Data;
    y = -simulation_data.InputPort(5).Data;
    z = -simulation_data.InputPort(6).Data;
    phi = simulation_data.InputPort(7).Data;
    the = simulation_data.InputPort(8).Data;
    psi = simulation_data.InputPort(9).Data;
    time = simulation_data.InputPort(19).Data;
    %Data output
    set(handles.roll, 'String', num2str(the));
    set(handles.pitch, 'String', num2str(phi));
    set(handles.yaw, 'String', num2str(psi));
    set(handles.X, 'String', num2str(x));
    set(handles.Y, 'String', num2str(y));
    set(handles.Z, 'String', num2str(z));
    set(handles.time, 'String', num2str(time));
    % Check landing
    if ((abs(x) > (xlim - 0)) || (abs(y) > (ylim - 0)) || (z < -2) || (z > (zlim - 1)))
        set_param('multi_model','SimulationCommand','Stop');
        % Correct display
        if(z < 0)
            z = 0;
        end
    end
end

function listbox1_Callback(hObject, eventdata, handles)

function listbox1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function popupmenu1_Callback(hObject, eventdata, handles)

function popupmenu1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function popupmenu2_Callback(hObject, eventdata, handles)

function popupmenu2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit1_Callback(hObject, eventdata, handles)

function edit1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit2_Callback(hObject, eventdata, handles)

function edit2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit3_Callback(hObject, eventdata, handles)

function edit3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function reset_Callback(hObject, eventdata, handles)
global x y z the phi psi time
if (strcmp(get_param('multi_model', 'SimulationStatus'), 'stopped'))
    x = 0;
    y = 0;
    z = 0;
    the = 0;
    phi = 0;
    psi = 0;
    time = 0;
end
function connect_button_ButtonDownFcn(hObject, eventdata, handles)



function time_Callback(hObject, eventdata, handles)
% hObject    handle to time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of time as text
%        str2double(get(hObject,'String')) returns contents of time as a double


% --- Executes during object creation, after setting all properties.
function time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function AZslider_Callback(hObject, eventdata, handles)
handles.AZval = get(hObject,'Value')*360; % Set azimuth view in degrees
handles.ELval = get(handles.ELslider,'Value')*90; % Set elevation view in degrees
view(handles.attitude_axes,handles.AZval,handles.ELval);
view(handles.position_axes,handles.AZval,handles.ELval);
drawnow
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function AZslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AZslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function ELslider_Callback(hObject, eventdata, handles)
handles.ELval = get(hObject,'Value')*90; % Set elevation view in degrees
handles.AZval = get(handles.AZslider,'Value')*360; % Set aximuth view in degrees
view(handles.attitude_axes,handles.AZval,handles.ELval);
view(handles.position_axes,handles.AZval,handles.ELval);
drawnow
guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function ELslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ELslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in defaultView.
function defaultView_Callback(hObject, eventdata, handles)
set(handles.ELslider,'Value',25/90);
set(handles.AZslider,'Value',234/360);
handles.AZval = get(handles.AZslider,'Value')*360; % Get azimuth view in degrees
handles.ELval = get(handles.ELslider,'Value')*90;
view(handles.attitude_axes,handles.AZval,handles.ELval);
view(handles.position_axes,handles.AZval,handles.ELval);
drawnow
guidata(hObject,handles);

% --- Executes on button press in XYview.
function XYview_Callback(hObject, eventdata, handles)
set(handles.ELslider,'Value',1);
set(handles.AZslider,'Value',0);
view(handles.attitude_axes,0,90);
view(handles.position_axes,0,90);
drawnow
guidata(hObject,handles);

% --- Executes on button press in XZview.
function XZview_Callback(hObject, eventdata, handles)
set(handles.ELslider,'Value',0);
set(handles.AZslider,'Value',0);
view(handles.attitude_axes,0,0);
view(handles.position_axes,0,0);
drawnow
guidata(hObject,handles);

% --- Executes on button press in YZview.
function YZview_Callback(hObject, eventdata, handles)
set(handles.ELslider,'Value',0);
set(handles.AZslider,'Value',90/360);
view(handles.attitude_axes,90,0);
view(handles.position_axes,90,0);
drawnow
guidata(hObject,handles);

% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
stop(handles.input_data_timer);
stop(handles.output_data_timer);
stop(handles.plot_timer);
set_param('multi_model','SimulationCommand','Stop');
close_serial();
close all;
clear all;
clc;


% --- Executes on button press in calibRC.
function calibRC_Callback(hObject, eventdata, handles)
if strcmp(get(handles.connect_button,'String'),'Disconnect RC')
    RollPitchCalibrationDialogue;
else
    errordlg('Please connect RC first')
end

% --- Executes on button press in start_button.
function start_button_Callback(hObject, eventdata, handles)
if (get(handles.pushbutton14,'Value') == 1)    
        if strcmp(get(hObject,'String'),'Start')
            set(hObject,'String','Starting');
            pause(0.2);
            set_param('multi_model','SimulationCommand','Start');
            pause(0.5);
            set_param('multi_model','SimulationCommand','Stop');
            pause(0.5);
            set_param('multi_model','SimulationCommand','Start');
            start(handles.input_data_timer);
            start(handles.output_data_timer);
            start(handles.plot_timer);
            set(hObject,'String','Stop');
            set(hObject, 'BackgroundColor', 'r');
        else
            stop(handles.input_data_timer);
            stop(handles.output_data_timer);
            stop(handles.plot_timer);
            set_param('multi_model','SimulationCommand','Stop');
            set(hObject,'String','Start');
            set(hObject, 'BackgroundColor', 'g');
        end
else
%     if strcmp(get(handles.connect_button,'String'),'Disconnect RC')
%         if strcmp(get(hObject,'String'),'Start')
%             set(hObject,'String','Starting');
%             pause(0.2);
%             set_param('multi_model','SimulationCommand','Start');
%             pause(0.5);
%             set_param('multi_model','SimulationCommand','Stop');            
%             start(handles.input_data_timer);
%             start(handles.output_data_timer);
%             start(handles.plot_timer);
%             set(hObject,'String','Stop');
%             set(hObject, 'BackgroundColor', 'r');
%         else
%             stop(handles.input_data_timer);
%             stop(handles.output_data_timer);
%             stop(handles.plot_timer);
%             set_param('multi_model','SimulationCommand','Stop');
%             set(hObject,'String','Start');
%             set(hObject, 'BackgroundColor', 'g');
%         end
%     else
%         errordlg('Please connect RC first')
%     end
    if strcmp(get(hObject,'String'),'Start')
        set(hObject,'String','Starting');
        pause(0.2);
        set_param('multi_model','SimulationCommand','Start');
        pause(0.5);
        set_param('multi_model','SimulationCommand','Stop');            
        start(handles.input_data_timer);
        start(handles.output_data_timer);
        start(handles.plot_timer);
        set(hObject,'String','Stop');
        set(hObject, 'BackgroundColor', 'r');
    else
        stop(handles.input_data_timer);
        stop(handles.output_data_timer);
        stop(handles.plot_timer);
        set_param('multi_model','SimulationCommand','Stop');
        set(hObject,'String','Start');
        set(hObject, 'BackgroundColor', 'g');
    end
end
% 
% --- Executes during object creation, after setting all properties.
function start_button_CreateFcn(hObject, eventdata, handles)
% hObject    handle to start_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
if strcmp(get(handles.start_button,'String'),'Start')
    delete(hObject);
else
    errordlg('Please stop simulation before close GUI window')
end


% --- Executes on button press in togglebutton1.
function togglebutton1_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton1


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
if (get(hObject,'Value') == 1)    
    set(hObject, 'BackgroundColor', 'r');
else
    set(hObject, 'BackgroundColor', 'g');
end
guidata(hObject, handles);
