---
description: "mergeWith est un Pipeable Operator qui souscrit simultanément à l'Observable original et à d'autres Observables pour les combiner en parallèle. Peut être utilisé pour intégrer plusieurs sources d'événements et les traiter en temps réel. Explique la différence avec merge() et l'implémentation type-safe avec TypeScript."
---

# mergeWith - Fusionner en Parallèle

L'opérateur `mergeWith` **souscrit simultanément** à l'Observable original et aux autres Observables spécifiés,
intégrant les valeurs émises par chacun en temps réel.
C'est la version Pipeable Operator de la Creation Function `merge`.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `Flux 1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `Flux 2: ${val}`),
  take(2)
);

source1$
  .pipe(mergeWith(source2$))
  .subscribe(console.log);

// Exemple de sortie:
// Flux 1: 0
// Flux 2: 0
// Flux 1: 1
// Flux 1: 2
// Flux 2: 1
```

- Souscrit à tous les Observables simultanément, les valeurs arrivent **dans l'ordre d'émission**.
- L'ordre n'est pas garanti, **dépend du timing d'émission de chaque Observable**.

[🌐 Documentation officielle RxJS - `mergeWith`](https://rxjs.dev/api/operators/mergeWith)


## 💡 Patterns d'utilisation typiques

- **Intégrer plusieurs sources d'événements** : Intégration d'actions utilisateur et de mises à jour automatiques
- **Combiner des récupérations de données parallèles** : Agréger les réponses de plusieurs APIs en un seul flux
- **Fusionner des mises à jour temps réel** : Intégrer WebSocket et polling


## 🧠 Exemple de code pratique (avec UI)

Un exemple intégrant les événements de clic utilisateur et un timer de mise à jour automatique pour afficher des notifications.

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

// Création de la zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Exemple pratique de mergeWith :</h3>';
document.body.appendChild(output);

// Création du bouton
const button = document.createElement('button');
button.textContent = 'Mise à jour manuelle';
document.body.appendChild(button);

// Flux de clics
const manualUpdate$ = fromEvent(button, 'click').pipe(
  map(() => '👆 Mise à jour manuelle exécutée')
);

// Timer de mise à jour automatique (toutes les 5 secondes)
const autoUpdate$ = interval(5000).pipe(
  map(val => `🔄 Mise à jour auto #${val + 1}`),
  take(3)
);

// Intégrer les deux et afficher
manualUpdate$
  .pipe(mergeWith(autoUpdate$))
  .subscribe((value) => {
    const timestamp = new Date().toLocaleTimeString();
    const item = document.createElement('div');
    item.textContent = `[${timestamp}] ${value}`;
    output.appendChild(item);
  });
```

- Un clic sur le bouton affiche immédiatement la mise à jour manuelle,
- Les mises à jour automatiques s'exécutent en parallèle toutes les 5 secondes.
- Les deux événements sont intégrés en temps réel.


## 🔄 Différence avec la Creation Function `merge`

### Différences de base

| | `merge` (Creation Function) | `mergeWith` (Pipeable Operator) |
|:---|:---|:---|
| **Lieu d'utilisation** | Utilisé comme fonction indépendante | Utilisé dans la chaîne `.pipe()` |
| **Syntaxe** | `merge(obs1$, obs2$, obs3$)` | `obs1$.pipe(mergeWith(obs2$, obs3$))` |
| **Premier flux** | Traité également | Traité comme flux principal |
| **Avantage** | Simple et lisible | Facile à combiner avec d'autres opérateurs |

### Exemples concrets de choix

**Pour une simple fusion, la Creation Function est recommandée**

```ts
import { merge, fromEvent } from 'rxjs';
import { map } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(map(() => 'Clic'));
const moves$ = fromEvent(document, 'mousemove').pipe(map(() => 'Mouvement souris'));
const keypress$ = fromEvent(document, 'keypress').pipe(map(() => 'Touche pressée'));

// Simple et lisible
merge(clicks$, moves$, keypress$).subscribe(console.log);
// Sortie: affiché dans l'ordre d'occurrence des événements
```

**Pour ajouter des transformations au flux principal, le Pipeable Operator est recommandé**

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, filter, throttleTime } from 'rxjs';

const userClicks$ = fromEvent(document, 'click');
const autoRefresh$ = interval(30000); // Toutes les 30 secondes

// ✅ Version Pipeable Operator - complète en un seul pipeline
userClicks$
  .pipe(
    throttleTime(1000),           // Prévention des clics répétés
    map(() => ({ source: 'user', timestamp: Date.now() })),
    mergeWith(
      autoRefresh$.pipe(
        map(() => ({ source: 'auto', timestamp: Date.now() }))
      )
    ),
    filter(event => event.timestamp > Date.now() - 60000)  // Dernière minute seulement
  )
  .subscribe(event => {
    console.log(`Mise à jour ${event.source}: ${new Date(event.timestamp).toLocaleTimeString()}`);
  });
```

### Résumé

- **`merge`** : Optimal pour simplement intégrer plusieurs flux également
- **`mergeWith`** : Optimal quand vous voulez ajouter des transformations au flux principal et intégrer d'autres flux


## ⚠️ Points d'attention

### Timing de complétion

Le flux combiné ne se termine pas tant que tous les Observables ne sont pas terminés.

```ts
import { of, interval, NEVER } from 'rxjs';
import { mergeWith, take } from 'rxjs';

of(1, 2, 3).pipe(
  mergeWith(
    interval(1000).pipe(take(2)),
    // NEVER  // ← ajouter ceci ne terminerait jamais
  )
).subscribe({
  next: console.log,
  complete: () => console.log('✅ Terminé')
});
// Sortie: 1 → 2 → 3 → 0 → 1 → ✅ Terminé
```

### Contrôle du nombre d'exécutions parallèles

Par défaut, tous les flux sont exécutés simultanément, mais peut être contrôlé avec `mergeMap`.

```ts
import { from, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

from([1, 2, 3, 4, 5]).pipe(
  mergeMap(
    val => of(val).pipe(delay(1000)),
    2  // Maximum 2 en parallèle
  )
).subscribe(console.log);
```

### Gestion des erreurs

Si une erreur se produit dans l'un des Observables, l'ensemble se termine en erreur.

```ts
import { throwError, interval } from 'rxjs';
import { mergeWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  mergeWith(
    throwError(() => new Error('Erreur survenue')).pipe(
      catchError((err: unknown) => of('Erreur récupérée'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Erreur:', err.message)
});
// Sortie: 0 → Erreur récupérée → 1
```


## 📚 Opérateurs associés

- **[merge](/fr/guide/creation-functions/combination/merge)** - Version Creation Function
- **[concatWith](/fr/guide/operators/combination/concatWith)** - Version Pipeable pour combinaison séquentielle
- **[mergeMap](/fr/guide/operators/transformation/mergeMap)** - Mapping parallèle de chaque valeur
