---
description: "raceWith est un opérateur de combinaison RxJS qui adopte uniquement le flux le plus rapide entre l'Observable original et d'autres Observables. Idéal pour l'implémentation de timeout, le traitement de fallback, l'adoption du plus rapide parmi plusieurs sources de données. Version pipeable operator pratique à utiliser dans les pipelines."
---

# raceWith - Adopter le flux le plus rapide dans un pipeline

L'opérateur `raceWith` **adopte uniquement l'Observable qui émet en premier** entre l'Observable original et les autres Observables spécifiés,
puis ignore les autres par la suite.
C'est la version Pipeable Operator de la Creation Function `race`.

## Syntaxe de base et utilisation

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Lent (5s)'));
const medium$ = timer(3000).pipe(map(() => 'Normal (3s)'));
const fast$ = timer(2000).pipe(map(() => 'Rapide (2s)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Sortie: Rapide (2s)
```

- Seul l'Observable qui émet en premier (dans cet exemple `fast$`) devient le gagnant, et son flux continue.
- Les autres Observables sont ignorés.

[Documentation officielle RxJS - `raceWith`](https://rxjs.dev/api/operators/raceWith)


## Patterns d'utilisation typiques

- **Implémentation de timeout**: Mettre en compétition le traitement principal et un timer de timeout
- **Traitement de fallback**: Adopter le plus rapide parmi plusieurs sources de données
- **Interaction utilisateur**: Adopter le premier entre un clic et une progression automatique


## Exemple de code pratique (avec UI)

Un exemple qui met en compétition un clic manuel et un timer de progression automatique, adoptant le plus rapide.

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

// Création de la zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Exemple pratique de raceWith:</h3>';
document.body.appendChild(output);

// Création du bouton
const button = document.createElement('button');
button.textContent = 'Avancer manuellement (cliquer dans les 5 secondes)';
document.body.appendChild(button);

// Message d'attente
const waiting = document.createElement('div');
waiting.textContent = 'Cliquez sur le bouton dans les 5 secondes, ou attendez la progression automatique...';
waiting.style.marginTop = '10px';
output.appendChild(waiting);

// Flux de clic manuel
const manualClick$ = fromEvent(button, 'click').pipe(
  take(1),
  map(() => '👆 Clic manuel sélectionné!')
);

// Timer de progression automatique (5 secondes)
const autoProgress$ = timer(5000).pipe(
  map(() => '⏰ Progression automatique sélectionnée!')
);

// Exécution de la course
manualClick$
  .pipe(raceWith(autoProgress$))
  .subscribe((winner) => {
    waiting.remove();
    button.disabled = true;

    const result = document.createElement('div');
    result.innerHTML = `<strong>${winner}</strong>`;
    result.style.color = 'green';
    result.style.fontSize = '18px';
    result.style.marginTop = '10px';
    output.appendChild(result);
  });
```

- Si vous cliquez sur le bouton dans les 5 secondes, le clic manuel est adopté.
- Si 5 secondes s'écoulent, la progression automatique est adoptée.
- **Le plus rapide gagne**, le plus lent est ignoré.


## Différence avec la Creation Function `race`

### Différences de base

| | `race` (Creation Function) | `raceWith` (Pipeable Operator) |
|:---|:---|:---|
| **Lieu d'utilisation** | Utilisé comme fonction indépendante | Utilisé dans la chaîne `.pipe()` |
| **Syntaxe** | `race(obs1$, obs2$, obs3$)` | `obs1$.pipe(raceWith(obs2$, obs3$))` |
| **Premier flux** | Traité également | Traité comme flux principal |
| **Avantage** | Simple et lisible | Facile à combiner avec d'autres opérateurs |

### Exemples concrets de choix

**Pour une simple course, la Creation Function est recommandée**

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const server1$ = timer(3000).pipe(map(() => 'Réponse du serveur 1'));
const server2$ = timer(2000).pipe(map(() => 'Réponse du serveur 2'));
const server3$ = timer(4000).pipe(map(() => 'Réponse du serveur 3'));

// Simple et lisible
race(server1$, server2$, server3$).subscribe(response => {
  console.log('Adopté:', response);
});
// Sortie: Adopté: Réponse du serveur 2 (le plus rapide à 2 secondes)
```

**Pour ajouter des transformations au flux principal, le Pipeable Operator est recommandé**

```ts
import { fromEvent, timer, of } from 'rxjs';
import { raceWith, map, switchMap, catchError } from 'rxjs';

const searchButton = document.createElement('button');
searchButton.textContent = 'Rechercher';
document.body.appendChild(searchButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Flux principal: requête de recherche utilisateur
const userSearch$ = fromEvent(searchButton, 'click').pipe(
  switchMap(() => {
    output.textContent = 'Recherche en cours...';

    // Simule un appel API (prend 3 secondes)
    return timer(3000).pipe(
      map(() => '🔍 Résultats de recherche: 100 correspondances'),
      catchError((err: unknown) => of('❌ Une erreur est survenue'))
    );
  })
);

// ✅ Version Pipeable Operator - complète en un seul pipeline
userSearch$
  .pipe(
    raceWith(
      // Timeout (2 secondes)
      timer(2000).pipe(
        map(() => '⏱️ Timeout: La recherche prend du temps')
      )
    )
  )
  .subscribe(result => {
    output.textContent = result;
  });
```

### Résumé

- **`race`**: Optimal pour simplement adopter le plus rapide parmi plusieurs flux
- **`raceWith`**: Optimal quand vous voulez implémenter un timeout ou un fallback tout en ajoutant des transformations au flux principal


## Points d'attention

### Exemple d'implémentation de timeout

Implémentation d'un traitement de timeout avec `raceWith`

```ts
import { of, timer, throwError } from 'rxjs';
import { raceWith, delay, mergeMap } from 'rxjs';

// Traitement lent (3 secondes)
const slowRequest$ = of('Données récupérées avec succès').pipe(delay(3000));

// Timeout (2 secondes)
const timeout$ = timer(2000).pipe(
  mergeMap(() => throwError(() => new Error('Timeout')))
);

slowRequest$
  .pipe(raceWith(timeout$))
  .subscribe({
    next: console.log,
    error: err => console.error(err.message)
  });
// Sortie: Timeout
```

### Tous les flux sont souscrits

`raceWith` souscrit à tous les Observables jusqu'à ce qu'un gagnant soit déterminé.
Une fois le gagnant déterminé, les Observables perdants sont automatiquement désabonnés.

```ts
import { timer } from 'rxjs';
import { raceWith, tap, map } from 'rxjs';

const slow$ = timer(3000).pipe(
  tap(() => console.log('slow$ déclenché')),
  map(() => 'slow')
);

const fast$ = timer(1000).pipe(
  tap(() => console.log('fast$ déclenché')),
  map(() => 'fast')
);

slow$.pipe(raceWith(fast$)).subscribe(console.log);
// Sortie:
// fast$ déclenché
// fast
// (slow$ est désabonné à 1 seconde, ne se déclenche pas à 3 secondes)
```

### Cas d'Observables synchrones

Si tous émettent de manière synchrone, le premier enregistré devient le gagnant.

```ts
import { of } from 'rxjs';
import { raceWith } from 'rxjs';

of('A').pipe(
  raceWith(of('B'), of('C'))
).subscribe(console.log);
// Sortie: A (parce qu'enregistré en premier)
```


## Opérateurs associés

- **[race](/fr/guide/creation-functions/selection/race)** - Version Creation Function
- **[timeout](/fr/guide/operators/utility/timeout)** - Opérateur dédié au timeout
- **[mergeWith](/fr/guide/operators/combination/mergeWith)** - Fusionne tous les flux

