classdef LatentResults
    %LATENTRESULTS Stores data from various UR5 motions

    properties
        forces % in N
        positions % x, y,depth, anglex, angley
        times
        measurements
    end

    methods
        function obj = LatentResults(filename)
            %LatentResults constructor: load results from path, or return NaN
            try
                % Extract arduino measurements
                lines = readlines("Data/"+filename+"_Arduino.log");
                lines = lines(3:end-1);
                arduinotimes(length(lines)) = datetime();
                arduinoreadings = [];
                for i = 1:length(lines)
                    line = char(lines(i));
                    arduinotimes(i) = datetime(line(2:24));
                    arduinoreadings = [arduinoreadings; str2double(split(line(27:end), ", ")).'];
                end

                % Extract scale measurements
                lines = readlines("Data/"+filename+"_scales.log");
                lines = lines(3:end-1);
                scaletimes(length(lines)) = datetime();
                scalereadings = zeros([length(lines), 1]);
                for i = 1:length(lines)
                    line = char(lines(i));
                    scaletimes(i) = datetime(line(2:24));
                    scalereadings(i) = 0.00981*str2double(line(30:end-4));
                end

                % Extract UR5 positions
                lines = readlines("Data/"+filename+"_positions.txt");
                lines = lines(2:end-1);
                postimes(length(lines)) = datetime();
                posreadings = zeros([length(lines), 5]);
                for i = 1:length(lines)
                    line = char(lines(i));
                    postimes(i) = datetime(line(1:24), "InputFormat", "MM/dd/uuuu, HH:mm:ss:SSS");
                    vals = str2double(split(line, (", ")));
                    posreadings(i, :) = vals(3:7).';
                end

                %Scale data is slowest: interpolate others to match
                [scaletimes, ia, ~] = unique(scaletimes);
                obj.times = seconds(scaletimes-scaletimes(1)).';
                obj.forces = scalereadings(ia);

                temppositions = zeros([length(scaletimes), 3]);
                [postimes, ia, ~] = unique(postimes);
                posreadings = posreadings(ia, :);
                for i = 1:size(posreadings, 2)
                    temppositions(:, i) = interp1(postimes,posreadings(:, i),scaletimes);
                end
                obj.positions = temppositions;

                tempmeas = zeros([length(scaletimes), size(arduinoreadings, 2)]);
                [arduinotimes, ia, ~] = unique(arduinotimes);
                arduinoreadings = arduinoreadings(ia, :);
                for i = 1:size(arduinoreadings, 2)
                    tempmeas(:, i) = interp1(arduinotimes,arduinoreadings(:, i),scaletimes);
                end
                obj.measurements = tempmeas;

            catch
                fprintf("Couldn't load results, returning NaN: "+filename+"\n");
                obj.forces = NaN;
                obj.positions = NaN;
                obj.times = NaN;
                obj.measurements = NaN;
            end

        end

        function plotall(obj)
            %PLOTALL with time

            subplot(3,1,1);
            plot(obj.times, obj.positions);
            xlim([0 obj.times(end)]);
            title("Positions");
            subplot(3,1,2);
            plot(obj.times, obj.forces);
            xlim([0 obj.times(end)]);
            title("Forces");
            subplot(3,1,3);
            plot(obj.times, obj.measurements);
            xlim([0 obj.times(end)]);
            title("Measurements");
            xlabel("Time (s)");
            set(gcf, 'color', 'w', 'position', [480 108 560 735]);
        end
    end

end