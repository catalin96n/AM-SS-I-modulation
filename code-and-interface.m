function varargout = fig12(varargin)



% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @fig12_OpeningFcn, ...
                   'gui_OutputFcn',  @fig12_OutputFcn, ...
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
% End initialization code - DO NOT EDIT


% --- Executes just before fig12 is made visible.
function fig12_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to fig12 (see VARARGIN)

% Choose default command line output for fig12
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes fig12 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = fig12_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double


% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

clear all;
close all;
clc;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


A=get(handles.edit2,'String');      %amplitudine sinus1
B=get(handles.edit4,'String');       %amplitudine sinus2
P=get(handles.edit6,'String');       %amplitudine purtatoare

f1=get(handles.edit11,'String');        %frecventa sinus1
f2=get(handles.edit12,'String');        %frecventa sinus2
f0=get(handles.edit13,'String');    %frecventa purtatoare

A=str2double(A);
B=str2double(B);
P=str2double(P);
f1=str2double(f1);
f2=str2double(f2);
f0=str2double(f0);


fp1=50000;       %frecventa purtatoare1 (schema)
fp2=f0-fp1;      %frecventa purtatoare2 (schema)

Fs=10*f0;                %frecventa de esantionare
tstop=max(1/f1,1/f2);    %timpul de afisare           
t=0:1/Fs:tstop;          %timpul

%----------------------------%
%----------EMITATOR----------%
%----------------------------%

%cele 2 sinusuri care formeaza semnalul de la intrare
sinus1=A*sin(2*pi*f1*t);
sinus2=A*sin(2*pi*f2*t);
%semnalul de la intrare
g=sinus1+sinus2;

%cele 2 purtatoare (schema)
p1=sqrt(P)*cos(2*pi*fp1*t);
p2=sqrt(P)*cos(2*pi*fp2*t);

%cele 2 filtre
%f1
wp_f1=2*pi*[fp1-max(f1,f2),fp1];
ws_f1=2*pi*[1,fp1*2];
Rp1=1;
Rs1=40;
[n1,w_f1]=ellipord(wp_f1,ws_f1,Rp1,Rs1,'s');
f_taiere_ftb1=w_f1/(2*pi)
[b1,a1]=ellip(n1,Rp1,Rs1,w_f1,'bandpass','s');
[bd1,ad1]=impinvar(b1,a1,Fs);
%f2
wp_f2=2*pi*[fp1-max(f1,f2),fp1];
ws_f2=2*pi*[(fp1-max(f1,f2))*0.5,fp1*1.5];
Rp2=0.5;
Rs2=50;
[n2,w_f2]=ellipord(wp_f2,ws_f2,Rp2,Rs2,'s');
f_taiere_ftb2=w_f2/(2*pi)
[b2,a2]=ellip(n2,Rp2,Rs2,w_f2,'bandpass','s');
[bd2,ad2]=impinvar(b2,a2,Fs);

%cele 2 semnale (dupa prima conversie si dupa a doua conversie) (schema)
s1=g.*p1;
    %s1=filter(bd1,ad1,s1);
s2=s1.*p2;
    %s2=filter(bd2,ad2,s2);
semnal_e=s2; %semnalul modulat, emis

%------------------------------------------%
%----------CANALUL DE COMUNICATIE----------%
%------------------------------------------%

RSZ_ON=get(handles.checkbox2, 'Value');
%valoarea RSZ, in dB
RSZ=get(handles.edit14,'String');  
RSZ=str2double(RSZ);
%adaugarea ZAGA peste semnal 
if RSZ_ON 
semnal_r=awgn(semnal_e,RSZ); %semnalul modulat, receptat
else
semnal_r=semnal_e;
end

%----------------------------%
%----------RECEPTOR----------%
%----------------------------%

%demodulator produs
p0=cos(2*pi*f0*t);
semnal_d=semnal_r.*p0;


%filtru FTJ
wp_f3=2*pi*1;  
ws_f3=2*pi*f0-wp_f3;
Rp3=1;
Rs3=40;
[n3,wt3]=buttord(wp_f3,ws_f3,Rp3,Rs3,'s');
frecv_taiere_f3=wt3/(2*pi)
[b3,a3]=butter(n3,wt3,'s');
[bd3,ad3]=impinvar(b3,a3,Fs);

%filtrarea FTJ
semnal_d_ftj=4*filter(bd3,ad3,semnal_d);


%---------------------------%
%----------ERORI------------%
%---------------------------%
%eroarea absoluta
e_abs=abs(g-semnal_d_ftj);
%eroarea relativa
e_rel=(e_abs./abs(g))*100;



%---------------------------%
%----------SPECTRE----------%
%---------------------------%

N=length(t)*10;
f=(0:Fs/N:Fs-1);
fshift=(-Fs/2:Fs/N:Fs/2-1);

%spectrul semnalului dupa prima conversie
spectru_s1=fft(s1,N)*1/N;
%spectrul semnalului dupa a doua conversie (semnal BLU emis)
spectru_e=fft(semnal_e,N)*1/N;
%spectrul semnalului BLU afectat de zgomot (semnal BLU receptat)
spectru_r=fft(semnal_r,N)*1/N;
%spectrul semnalului dupa demodulatorul de produs
spectru_d=fft(semnal_d,N)*1/N;
%spectrul semnalului dupa demodulatorul de produs si FTJ
spectru_d_ftj=fft(semnal_d_ftj,N)*1/N;


%-----------------------------%
%----------REZULTATE----------%
%-----------------------------%

figure(1);
subplot(2,1,1); 
plot(t,sinus1); hold on; plot(t,sinus2);
title('Cele doua sinusuri care formeaza semnalul de la intrare');
xlabel('t[s]'); ylabel('Amplitudine');
legend('sinus1 (f1)','sinus2 (f2)');
subplot(2,1,2); 
plot(t,g);
title('Semnalul de la intrare');
xlabel('t[s]'); ylabel('Amplitudine');

figure(2);
subplot(3,1,1);
plot(t,g);
title('Semnalul de la intrare');
xlabel('t[s]'); ylabel('Amplitudine');
subplot(3,1,2);
plot(t,s1);
title('Semnalul modulat, dupa prima conversie');
xlabel('t[s]'); ylabel('Amplitudine');
subplot(3,1,3);
plot(t,semnal_e);
title('Semnalul modulat, dupa a doua conversie = Semnalul BLU');
xlabel('t[s]'); ylabel('Amplitudine');

figure(3);
subplot(2,1,1);
plot(fshift, abs(fftshift(spectru_s1)));
title('Spectrul semnalului modulat - dupa prima conversie');
xlabel('f[Hz]'); ylabel('Amplitudine');
subplot(2,1,2);
plot(fshift, abs(fftshift(spectru_e)));
title('Spectrul semnalului modulat - dupa a 2a conversie');
xlabel('f[Hz]'); ylabel('Amplitudine');

figure(4);
subplot(2,1,1);
plot(t,semnal_e);
title('Semnalul BLU emis, neafectat de zgomot');
xlabel('t[s]'); ylabel('Amplitudine');
subplot(2,1,2);
plot(t,semnal_r);
title('Semnalul BLU receptat, afectat de zgomot');
xlabel('t[s]'); ylabel('Amplitudine');

figure(5);
subplot(2,1,1);
plot(fshift, abs(fftshift(spectru_e)));
title('Spectrul semnalului emis');
xlabel('f[Hz]'); ylabel('Amplitudine');
subplot(2,1,2);
plot(fshift, abs(fftshift(spectru_r)));
title('Spectrul semnalului receptat');
xlabel('f[Hz]'); ylabel('Amplitudine');

figure(6);
subplot(3,1,1);
plot(fshift, abs(fftshift(spectru_r)));
title('Spectrul semnalului modulat receptat');
xlabel('f[Hz]'); ylabel('Amplitudine');
subplot(3,1,2);
plot(fshift, abs(fftshift(spectru_d)));
title('Spectrul semnalului modulat, dupa demodulare produs');
xlabel('f[Hz]'); ylabel('Amplitudine');
subplot(3,1,3)
plot(fshift, abs(fftshift(spectru_d_ftj)));
title('Spectrul semnalului modulat, dupa demodulare produs si FTJ');
xlabel('f[Hz]'); ylabel('Amplitudine');

figure(7)
subplot(3,1,1)
plot(t,semnal_d_ftj)
title('Semnalul demodulat');
xlabel('t[s]'); ylabel('Amplitudine');
subplot(3,1,2)
plot(t,semnal_d_ftj,'.')
hold on 
plot(t,semnal_r)
title('Semnalul demodulat VS semnalul BLU receptat');
xlabel('t[s]'); ylabel('Amplitudine');
legend('semnal demodulat','semnal BLU receptat');
subplot(3,1,3)
plot(t,semnal_d_ftj)
hold on 
plot(t,g)
title('Semnalul demodulat VS semnalul initial');
xlabel('t[s]'); ylabel('Amplitudine');
legend('semnal demodulat','semnal initial');

figure(8)
subplot(2,1,1)
plot(t,semnal_d_ftj)
hold on 
plot(t,g)
title('Semnalul demodulat VS semnalul initial');
xlabel('f[Hz]'); ylabel('Amplitudine');
legend('semnal demodulat','semnal initial');
subplot(2,1,2)
plot([1:1:length(t)],e_abs,'.')
title('Eroarea absoluta (semnal initial - semnal demodulat)');

