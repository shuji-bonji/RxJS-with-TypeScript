---
description: "window divise l'Observable source en Observables imbriqués au timing de l'émission d'un autre Observable. Idéal pour le traitement de flux avancé piloté par événements."
---

# window - Fenêtre par Événement

L'opérateur `window` regroupe les valeurs de l'Observable source **jusqu'à ce qu'un autre Observable émette des valeurs** et sort ce groupe **comme un nouvel Observable**.
Alors que `buffer` renvoie un tableau, `window` renvoie **Observable\<T>**, permettant d'appliquer d'autres opérateurs à chaque fenêtre.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

// Émet des valeurs toutes les 100ms
const source$ = interval(100);

// Utilise l'événement de clic comme déclencheur
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  window(clicks$),
  mergeAll() // Aplatit chaque fenêtre
).subscribe(value => {
  console.log('Valeur dans la fenêtre :', value);
});

// Une nouvelle fenêtre est créée à chaque clic
```

- Chaque fois que `clicks$` émet une valeur, une nouvelle fenêtre (Observable) est créée.
- Chaque fenêtre peut être traitée comme un Observable indépendant.

[🌐 Documentation officielle RxJS - `window`](https://rxjs.dev/api/operators/window)

## 💡 Patterns d'utilisation typiques

- Division de flux pilotée par événements
- Appliquer un traitement différent à chaque fenêtre
- Regroupement de données avec délimitation dynamique
- Traitement d'agrégation pour chaque fenêtre

## 🔍 Différences avec buffer

| Opérateur | Sortie | Cas d'utilisation |
|:---|:---|:---|
| `buffer` | **Tableau (T[])** | Traitement groupé des valeurs |
| `window` | **Observable\<T>** | Traitement de flux différent par groupe |

```ts
import { interval, timer } from 'rxjs';
import { buffer, window, mergeAll } from 'rxjs';

const source$ = interval(100);
const trigger$ = timer(1000, 1000);

// buffer - sortie en tableau
source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Buffer (tableau) :', values);
});

// window - sortie en Observable
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  console.log('Fenêtre (Observable) :', window$);
  window$.subscribe(value => {
    console.log('  Valeur dans la fenêtre :', value);
  });
});
```

## 🧠 Exemple de code pratique : comptage par fenêtre

Exemple de déclenchement par clic de bouton et comptage du nombre d'événements.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, map, mergeAll, scan } from 'rxjs';

// Création du bouton
const button = document.createElement('button');
button.textContent = 'Délimiter la fenêtre';
document.body.appendChild(button);

// Zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Émet des valeurs toutes les 100ms
const source$ = interval(100);

// Utilise le clic du bouton comme déclencheur
const clicks$ = fromEvent(button, 'click');

let windowCount = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const currentWindow = ++windowCount;
    console.log(`Fenêtre ${currentWindow} démarrée`);

    // Compte les valeurs de chaque fenêtre
    return window$.pipe(
      scan((count) => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  output.textContent = `Fenêtre actuelle : ${windowCount}, Comptage : ${count}`;
});
```

## ⚠️ Points d'attention

### 1. Gestion des abonnements aux fenêtres

Chaque fenêtre est un Observable indépendant et doit être explicitement souscrite.

```ts
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  // Sans souscrire à la fenêtre elle-même, les valeurs ne circulent pas
  window$.subscribe(value => {
    console.log('Valeur :', value);
  });
});
```

Ou utilisez `mergeAll()`, `concatAll()`, `switchAll()` pour aplatir.

### 2. Attention aux fuites de mémoire

**Problème** : Si l'Observable déclencheur n'émet pas de valeur, la première fenêtre reste ouverte indéfiniment et les valeurs s'accumulent à l'infini.

```ts
// ✅ Bon exemple : définir un timeout
const autoClose$ = timer(5000); // Valeur émise automatiquement après 5s
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$) // La fenêtre se ferme dans les 5 secondes
).subscribe();
```

### 3. Chevauchement des fenêtres

Par défaut, les fenêtres ne se chevauchent pas (la suivante démarre après la fermeture de la précédente).
Si le chevauchement est nécessaire, utilisez `windowToggle` ou `windowWhen`.

## 📚 Opérateurs associés

- [`buffer`](./buffer) - Regroupe les valeurs en tableau (version tableau de window)
- [`windowTime`](./windowTime) - Division de fenêtre basée sur le temps
- [`windowCount`](./windowCount) - Division de fenêtre basée sur le nombre
- [`windowToggle`](./windowToggle) - Contrôle de fenêtre avec Observables début/fin
- [`windowWhen`](./windowWhen) - Division de fenêtre avec condition de fermeture dynamique
- [`groupBy`](./groupBy) - Groupe les Observables par clé

## Résumé

L'opérateur `window` est un outil puissant qui peut diviser le flux déclenché par un Observable externe et traiter chaque groupe comme un Observable indépendant.

- ✅ Peut appliquer un traitement différent à chaque fenêtre
- ✅ Contrôle flexible piloté par événements
- ✅ Prend en charge les opérations de flux avancées
- ⚠️ Nécessite une gestion des abonnements
- ⚠️ Attention aux fuites de mémoire
