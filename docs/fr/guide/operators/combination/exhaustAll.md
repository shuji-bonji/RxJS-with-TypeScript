---
description: "exhaustAll est un opérateur qui reçoit un Higher-order Observable (Observable d'Observables) et ignore les nouveaux Observables internes pendant qu'un est en cours d'exécution."
---

# exhaustAll - Ignorer les nouveaux Observables internes pendant l'exécution

L'opérateur `exhaustAll` reçoit un **Higher-order Observable** (Observable d'Observables)
et **ignore les nouveaux Observables internes si un est en cours d'exécution**.

## Syntaxe de base et utilisation

```ts
import { fromEvent, interval } from 'rxjs';
import { map, exhaustAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Démarre un nouveau compteur à chaque clic (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Ignore les nouveaux clics si un compteur est en cours d'exécution
higherOrder$
  .pipe(exhaustAll())
  .subscribe(x => console.log(x));

// Sortie (avec 3 clics consécutifs):
// 0 (1er compteur)
// 1 (1er compteur)
// ← clic ici (ignoré: 1er en cours)
// 2 (1er compteur) ← complété
// ← clic ici (accepté: pas de compteur en cours)
// 0 (2ème compteur)
// 1 (2ème compteur)
// 2 (2ème compteur)
```

- Si un Observable interne est en cours d'exécution, **les nouveaux Observables internes sont ignorés**
- **Attend la complétion de l'Observable en cours avant d'accepter le suivant**
- Idéal pour prévenir les exécutions doubles

[Documentation officielle RxJS - `exhaustAll`](https://rxjs.dev/api/index/function/exhaustAll)

## Patterns d'utilisation typiques

- **Prévention du double-clic (prévention des clics répétés sur un bouton)**
- **Prévention de la double soumission de requêtes de connexion**
- **Prévention de l'exécution en double des opérations de sauvegarde**

## Exemple de code pratique

Un exemple qui empêche le double-clic sur un bouton de sauvegarde

```ts
import { fromEvent, of } from 'rxjs';
import { map, exhaustAll, delay } from 'rxjs';

const saveButton = document.createElement('button');
saveButton.textContent = 'Sauvegarder';
document.body.appendChild(saveButton);

const output = document.createElement('div');
document.body.appendChild(output);

let saveCount = 0;

// Événement de clic du bouton
const clicks$ = fromEvent(saveButton, 'click');

// Higher-order Observable: traitement de sauvegarde simulé pour chaque clic
const saves$ = clicks$.pipe(
  map(() => {
    const id = ++saveCount;
    const start = Date.now();

    // Désactive temporairement le bouton (feedback visuel)
    saveButton.disabled = true;

    // Traitement de sauvegarde simulé (délai de 2 secondes)
    return of(`Sauvegarde terminée #${id}`).pipe(
      delay(2000),
      map(msg => {
        saveButton.disabled = false;
        const elapsed = ((Date.now() - start) / 1000).toFixed(1);
        return `${msg} (${elapsed}s)`;
      })
    );
  }),
  exhaustAll() // Ignore les nouveaux clics pendant la sauvegarde
);

saves$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});

// Log des clics ignorés
clicks$.subscribe(() => {
  if (saveButton.disabled) {
    console.log('Clic ignoré car sauvegarde en cours');
  }
});
```

- Pendant le traitement de sauvegarde, **les nouveaux clics sont ignorés**
- Après la complétion de la sauvegarde, le prochain clic est accepté

## Opérateurs associés

| Opérateur | Description |
|---|---|
| `exhaustMap` | Raccourci pour `map` + `exhaustAll` (couramment utilisé) |
| [mergeAll](./mergeAll) | Souscrit à tous les Observables internes en parallèle |
| [concatAll](./concatAll) | Souscrit aux Observables internes séquentiellement (les met en file d'attente) |
| [switchAll](./switchAll) | Bascule vers le nouvel Observable interne (annule l'ancien) |

## Comparaison avec d'autres opérateurs

| Opérateur | Quand un nouvel Observable interne est émis |
|---|---|
| `mergeAll` | Exécution en parallèle |
| `concatAll` | Ajout à la file d'attente (attend la complétion du précédent) |
| `switchAll` | Annule l'ancien et bascule |
| `exhaustAll` | **Ignore (attend la complétion de l'actuel)** |

## Points d'attention

### Perte d'événements

`exhaustAll` **ignore complètement** les événements pendant l'exécution, donc c'est inapproprié si vous voulez traiter tous les événements.

```ts
// ❌ Si vous voulez enregistrer tous les clics, exhaustAll est inapproprié
// ✅ Utilisez mergeAll ou concatAll
```

### Feedback UI

Il est important de communiquer visuellement à l'utilisateur que "l'événement a été ignoré".

```ts
// Désactive le bouton
saveButton.disabled = true;

// Affiche un message toast
showToast('Traitement en cours. Veuillez patienter.');
```

### Cas d'utilisation appropriés

#### `exhaustAll` est optimal pour
- Traitement de connexion (prévention de la double soumission)
- Traitement de sauvegarde (prévention de l'exécution en double)
- Animation (ne pas démarrer une nouvelle animation pendant l'exécution)

#### `exhaustAll` est inapproprié pour
- Traitement de recherche (vous voulez exécuter la dernière recherche → `switchAll`)
- Traitement de tous les événements (→ `mergeAll` ou `concatAll`)

