% Mirollo–Strogatz
% φ += ε * (1 - φ);    // when flash detected
% φ = 0;               // when flashing

clear; clc; clf;

% Params
PHASE_MAX = 255;
EPSILON = 64;
PHASE_STEP = 1;
TICK_DELAY_MS = 2;
JUMP_TO_FLASH_MARGIN = 16;
JITTER_INTERVAL = 8;
seed = 1;

% Sim params
num_fireflies = 2;
len = 5000;

rng(seed);

% State arrays
phase = zeros(1, num_fireflies);
refractory = zeros(1, num_fireflies);
jitter_tick = zeros(1, num_fireflies);
last_rx = ones(1, num_fireflies);
flash_log = zeros(num_fireflies, len);
phase_log = zeros(num_fireflies, len);

% Initial desync
phase = phase + floor(rand(num_fireflies)*100);

for t = 1:len
    rx_now = ones(1, num_fireflies);
    
    for i = 1:num_fireflies
        % Jitter
        %jitter_tick(i) = jitter_tick(i) + 1;
        if jitter_tick(i) >= JITTER_INTERVAL
            jitter_tick(i) = 0;
            jitter = randi([0 1]);
            phase(i) = phase(i) + PHASE_STEP + jitter;
        else
            phase(i) = phase(i) + PHASE_STEP;
        end

        if phase(i) > PHASE_MAX
            phase(i) = PHASE_MAX;
        end
    end

    % Flash
    for i = 1:num_fireflies
        if phase(i) >= PHASE_MAX
            flash_log(i, t) = 1;
            phase(i) = 0;
            refractory(i) = 2;
        end
    end

    % Broadcast IR
    senders = find(flash_log(:, t) > 0);
    for sender = senders'
        for i = 1:num_fireflies
            if i == sender, continue; end
            rx_now(i) = 0;  % Pulse received
        end
    end

    % Sync logic
    for i = 1:num_fireflies
        if last_rx(i) && ~rx_now(i) && refractory(i) == 0
            % Only adjust teh lagging one
            if phase(i) > (PHASE_MAX / 4)
                delta = round((EPSILON * (PHASE_MAX - phase(i))) / PHASE_MAX);
                phase(i) = phase(i) + delta;
                if phase(i) > PHASE_MAX
                    phase(i) = PHASE_MAX;
                end

                % if (PHASE_MAX - phase(i)) < JUMP_TO_FLASH_MARGIN
                %     % Early flash due to sync
                %     flash_log(i, t) = 2;
                %     phase(i) = 0;
                %     refractory(i) = 20;
                % else
                %     refractory(i) = 10;
                % end
            end
        end

        if refractory(i) > 0
            refractory(i) = refractory(i) - 1;
        end

        last_rx(i) = rx_now(i);
        phase_log(i, t) = phase(i);
    end
end

% Plot
tvec = (1:len) * TICK_DELAY_MS;

figure;
subplot(2,1,1);
plot(tvec, phase_log');
title('Phases');
xlabel('Time (ms)');
ylabel('Phase (0–255)');

subplot(2,1,2);
hold on;
for i = 1:num_fireflies
    stem(tvec, i * (flash_log(i,:) > 0), 'Marker', 'none');
end
title('Flashes');
xlabel('Time (ms)');
ylabel('Firefly #');
yticks(1:num_fireflies);
ylim([0 num_fireflies+1]);
