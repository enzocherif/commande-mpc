% Définition des matrices du système
Ad = [0.5, 1, 0;
      -1, 0, 1;
      0.25, 0, 0];

Bd = [0.58;
      -1.06;
      0.45];

C = [1, 0, 0];
D = 0;

% % Calcul du polynôme caractéristique de Ad
% char_poly = poly(Ad);
% 
% % Calcul de la fonction de transfert (numérateur et dénominateur)
% [num, den] = ss2tf(Ad, Bd, C, D);
% 
% % Création de la fonction de transfert
% sys_tf = tf(num, den);
% 
% % Calcul des racines du polynôme caractéristique (valeurs propres)
% eigenvalues_Ad = roots(char_poly);
% 
% % Affichage des résultats
% disp('Polynôme caractéristique de Ad :');
% disp(char_poly);
% 
% disp('Valeurs propres de Ad (racines du polynôme caractéristique) :');
% disp(eigenvalues_Ad);
% 
% disp('Fonction de transfert :');
% disp(sys_tf);
% 
% % Calcul de la matrice d'observabilité
% Ob = obsv(Ad, C);
% 
% % Calcul du rang de la matrice d'observabilité
% rank_Ob = rank(Ob);
% 
% % Dimension du système
% n = size(Ad, 1);
% 
% % Affichage des résultats
% disp('Matrice d''observabilité :');
% disp(Ob);
% 
% disp('Rang de la matrice d''observabilité :');
% disp(rank_Ob);
% 
% % Vérification de l'observabilité
% if rank_Ob == n
%     disp('Le système est observable.');
% else
%     disp('Le système n''est pas observable.');
% end

% Définition des pôles souhaités pour l'observateur
poles_observer = [0.1, 0.1, 0.1];

% Calcul du gain de l'observateur K_obs avec acker
Kobs = acker(Ad', C', poles_observer)'; % Transpose pour le calcul correct

% Affichage des résultats
disp('Gain de l''observateur K_obs :');
disp(Kobs);

% Vérification de la matrice dynamique de l'observateur
A_tilde = Ad - Kobs * C;
disp('Nouvelle matrice dynamique A_tilde = Ad - K_obs * C :');
disp(A_tilde);

% Vérification des pôles de A_tilde
eigenvalues_A_tilde = eig(A_tilde);
disp('Valeurs propres de A_tilde (doivent correspondre aux pôles souhaités) :');
disp(eigenvalues_A_tilde);
