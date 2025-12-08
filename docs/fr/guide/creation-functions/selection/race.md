---
description: "La fonction de création race réalise un processus de concaténation spécial qui n'adopte que le premier flux qui émet une valeur parmi plusieurs Observables et ignore les autres par la suite."
---

# race - adopter le flux qui a émis la valeur en premier

`race` est une fonction de création concaténée spéciale qui utilise **uniquement le premier Observable qui émet une valeur** parmi plusieurs Observables, et ignore les autres Observables.


## Syntaxe de base et utilisation

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Lent (5 secondes)'));
const fast$ = timer(2000).pipe(map(() => 'Rapide (2 secondes)'));

race(slow$, fast$).subscribe(console.log);
// Sortie: Rapide (2 secondes)
```

- Seul l'Observable qui a émis la valeur en premier est gagnant et continue avec les flux suivants.
- Les autres Observables sont ignorés.

[🌐 Documentation officielle RxJS - `race`](https://rxjs.dev/api/index/function/race)


## Modèles d'utilisation typiques

- **Traiter la première de plusieurs actions de l'utilisateur (clics, frappes, défilement)**
- **Adopter le premier de plusieurs déclencheurs, tels que l'envoi manuel et l'enregistrement automatique**
- **Afficher les premières données complétées en premier parmi plusieurs processus d'acquisition de données**

## Exemples de code pratique (avec interface utilisateur)

Simulation d'une course pour adopter uniquement la première donnée issue de trois flux se déclenchant à des moments différents.

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Créer une zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Exemple pratique de race:</h3>';
document.body.appendChild(output);

// Observables avec des timings différents
const slow$ = timer(5000).pipe(map(() => 'Lent (après 5 secondes)'));
const medium$ = timer(3000).pipe(map(() => 'Moyen (après 3 secondes)'));
const fast$ = timer(2000).pipe(map(() => 'Rapide (après 2 secondes)'));

const startTime = Date.now();

// Message de début de course
const waiting = document.createElement('div');
waiting.textContent = 'Course démarrée... En attente du premier flux à émettre.';
output.appendChild(waiting);

// Exécuter la course
race(slow$, medium$, fast$).subscribe(winner => {
  const endTime = Date.now();
  const elapsed = ((endTime - startTime) / 1000).toFixed(2);

  const result = document.createElement('div');
  result.innerHTML = `<strong>Gagnant:</strong> ${winner} (Temps écoulé: ${elapsed} secondes)`;
  result.style.color = 'green';
  result.style.marginTop = '10px';
  output.appendChild(result);

  const explanation = document.createElement('div');
  explanation.textContent = '※ Seul le premier Observable qui a émis une valeur est sélectionné.';
  explanation.style.marginTop = '5px';
  output.appendChild(explanation);
});
```

- Après 2 secondes, le premier `fast$` est émis et par la suite, seul `fast$` est produit.
- Les autres émissions `medium$` et `slow$` seront ignorées.


## Opérateurs associés

- **[raceWith](/fr/guide/operators/combination/raceWith)** - Version opérateur Pipeable (utilisée dans le pipeline)
- **[timeout](/fr/guide/operators/utility/timeout)** - Opérateur de temporisation uniquement
- **[merge](/fr/guide/creation-functions/combination/merge)** - Fonction de création fusionnant tous les flux
