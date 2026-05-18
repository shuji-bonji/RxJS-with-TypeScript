---
description: "L'opérateur windowTime divise l'Observable à intervalles de temps réguliers, permettant de traiter les valeurs émises dans chaque fenêtre temporelle comme des Observables individuels. Explique la division de flux basée sur le temps, le traitement par lots, les différences avec bufferTime, et l'implémentation TypeScript type-safe."
---

# windowTime - Fenêtre Temporelle

L'opérateur `windowTime` regroupe les valeurs de l'Observable source **à intervalles réguliers** et produit ce groupe **comme un nouvel Observable**.
Alors que `bufferTime` renvoie un tableau, `windowTime` renvoie **Observable\\<T>**, permettant d'appliquer d'autres opérateurs à chaque fenêtre.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { windowTime, mergeAll, take } from 'rxjs';

// Émet des valeurs toutes les 100ms
const source$ = interval(100);

source$.pipe(
  windowTime(1000), // Crée une fenêtre toutes les secondes
  take(3),          // Seulement les 3 premières fenêtres
  mergeAll()        // Aplatit chaque fenêtre
).subscribe(value => {
  console.log('Valeur :', value);
});

// Sortie :
// 1ère seconde : 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// 2ème seconde : 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
// 3ème seconde : 20, 21, 22, 23, 24, 25, 26, 27, 28, 29
```

- Une nouvelle fenêtre (Observable) est créée à chaque temps spécifié (1000ms).
- Chaque fenêtre peut être traitée comme un Observable indépendant.

[🌐 Documentation officielle RxJS - `windowTime`](https://rxjs.dev/api/operators/windowTime)

## 💡 Patterns d'utilisation typiques

- **Traitement par lots basé sur le temps** : traitement groupé des données à intervalles réguliers
- **Agrégation de données en temps réel** : comptage du nombre d'événements par seconde
- **Surveillance des performances** : collecte de métriques à intervalles réguliers
- **Analyse de séries temporelles** : traitement statistique par fenêtre temporelle

## 🔍 Différences avec bufferTime

| Opérateur | Sortie | Cas d'utilisation |
|:---|:---|:---|
| `bufferTime` | **Tableau (T[])** | Traitement groupé des valeurs |
| `windowTime` | **Observable\\<T>** | Traitement de flux différent par fenêtre temporelle |

## 🧠 Exemple de code pratique : compter les clics par seconde

Exemple de comptage du nombre de clics sur un bouton toutes les secondes.

```ts
import { fromEvent } from 'rxjs';
import { windowTime, map, mergeAll, scan } from 'rxjs';

// Création du bouton
const button = document.createElement('button');
button.textContent = 'Cliquer';
document.body.appendChild(button);

// Zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Événements de clic
const clicks$ = fromEvent(button, 'click');

let windowNumber = 0;

clicks$.pipe(
  windowTime(1000), // Crée une fenêtre toutes les secondes
  map(window$ => {
    ++windowNumber;

    // Compte les clics dans chaque fenêtre
    return window$.pipe(
      scan(count => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  const timestamp = new Date().toLocaleTimeString();
  output.textContent = `[${timestamp}] Fenêtre ${windowNumber} : ${count} clics`;
});
```

## 📊 Fenêtres qui se chevauchent (windowCreationInterval)

Vous pouvez faire chevaucher les fenêtres en spécifiant `windowCreationInterval` comme second argument.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray, take, map } from 'rxjs';

const source$ = interval(100);

source$.pipe(
  windowTime(
    2000,  // Durée de la fenêtre : 2 secondes
    1000   // Intervalle de création de fenêtre : 1 seconde
  ),
  take(3),
  mergeMap((window$, index) =>
    window$.pipe(
      toArray(),
      map(values => ({ window: index + 1, values }))
    )
  )
).subscribe(result => {
  console.log(`Fenêtre ${result.window} : ${result.values.length} valeurs`);
});
```

**Explication du comportement :**
- **Fenêtre 1** : valeurs de 0s à 2s `[0, 1, 2, ..., 19]` (20 éléments)
- **Fenêtre 2** : valeurs de 1s à 3s `[10, 11, 12, ..., 29]` (20 éléments) ← valeurs 10-19 chevauchent avec fenêtre 1
- **Fenêtre 3** : valeurs de 2s à 4s `[20, 21, 22, ..., 39]` (20 éléments) ← valeurs 20-29 chevauchent avec fenêtre 2

## ⚠️ Points d'attention

### 1. Gestion des abonnements aux fenêtres

Chaque fenêtre est un Observable indépendant et doit être explicitement souscrite.

```ts
source$.pipe(
  windowTime(1000),
  mergeAll() // Fusionne toutes les fenêtres
).subscribe(value => {
  console.log('Valeur :', value);
});
```

### 2. Gestion de la mémoire

Pour les exécutions de longue durée, il est important de se désabonner correctement.

```ts
import { takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

const destroy$ = new Subject<void>();

source$.pipe(
  windowTime(1000),
  mergeAll(),
  takeUntil(destroy$) // Se désabonne à la destruction
).subscribe();

// À la destruction du composant, etc.
destroy$.next();
destroy$.complete();
```

### 3. Spécification de la taille maximale (maxWindowSize)

Vous pouvez limiter le nombre maximum de valeurs par fenêtre avec le 3ème argument.

```ts
interval(100).pipe(
  windowTime(
    2000,      // Durée de la fenêtre : 2 secondes
    undefined, // Intervalle de création : par défaut (pas de chevauchement)
    5          // Nombre max de valeurs : jusqu'à 5
  ),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Fenêtre :', values);
  // Contient un maximum de 5 valeurs
});
```

## 📚 Opérateurs associés

- **[bufferTime](./bufferTime)** - Regroupe les valeurs en tableau (version tableau de windowTime)
- **[window](./window)** - Division de fenêtre par émission d'Observable
- **[windowCount](./windowCount)** - Division de fenêtre basée sur le nombre
- **[windowToggle](./windowToggle)** - Contrôle de fenêtre avec Observables début/fin
- **[windowWhen](./windowWhen)** - Division de fenêtre avec condition de fermeture dynamique

## Résumé

L'opérateur `windowTime` est un outil puissant pour diviser les flux sur une base temporelle et traiter chaque fenêtre temporelle comme un Observable séparé.

- ✅ Crée automatiquement des fenêtres à intervalles réguliers
- ✅ Peut appliquer un traitement différent à chaque fenêtre
- ✅ Prend en charge les fenêtres glissantes (chevauchement)
- ✅ Idéal pour l'agrégation et l'analyse de données en temps réel
- ⚠️ Nécessite une gestion des abonnements
- ⚠️ Attention à la gestion de la mémoire
