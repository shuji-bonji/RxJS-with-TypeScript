---
description: "L'opérateur observeOn contrôle le timing d'émission des valeurs Observable en utilisant un planificateur spécifié, utile pour optimiser le traitement asynchrone et les animations."
---

# observeOn - Contrôle du contexte d'exécution

L'opérateur `observeOn` **contrôle le timing d'émission et le contexte d'exécution des valeurs Observable en utilisant un planificateur spécifié**. Il permet aux opérations suivantes dans le flux de s'exécuter sur un planificateur spécifique.

## 🔰 Syntaxe et comportement de base

Spécifiez un planificateur pour rendre les opérations suivantes asynchrones.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('Début');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(v => console.log('Valeur :', v));

console.log('Fin');

// Sortie :
// Début
// Fin
// Valeur : 1
// Valeur : 2
// Valeur : 3
```

Les opérations avant `observeOn` s'exécutent de manière synchrone, tandis que les opérations après `observeOn` s'exécutent sur le planificateur spécifié.

[🌐 Documentation officielle RxJS - observeOn](https://rxjs.dev/api/index/function/observeOn)

## 💡 Cas d'utilisation typiques

- **Éviter le blocage du thread UI** : Rendre le traitement lourd asynchrone
- **Optimisation des animations** : Rendu fluide avec `animationFrameScheduler`
- **Priorisation du traitement** : Contrôle du timing d'exécution avec différents planificateurs
- **Contrôle microtâche/macrotâche** : Ajustements fins du timing

## Types de planificateurs

| Planificateur | Caractéristiques | Cas d'utilisation |
|:---|:---|:---|
| `asyncScheduler` | Basé sur `setTimeout` | Traitement asynchrone général |
| `asapScheduler` | Microtâche (Promise.then) | Exécution asynchrone la plus rapide possible |
| `queueScheduler` | File d'attente synchrone | Optimisation du traitement récursif |
| `animationFrameScheduler` | `requestAnimationFrame` | Animation, rendu 60fps |

> [!TIP]
> Pour plus de détails sur les planificateurs, voir [Types et utilisation des planificateurs](/fr/guide/schedulers/types).

## 🧪 Exemple de code pratique 1 : Éviter le blocage de l'UI

Exemple de traitement de grandes quantités de données par lots de manière asynchrone.

```ts
import { range, asapScheduler } from 'rxjs';
import { observeOn, bufferCount, tap } from 'rxjs';

// Création de l'interface utilisateur
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'observeOn - Éviter le blocage de l\'UI';
container.appendChild(title);

const progress = document.createElement('div');
progress.style.marginBottom = '10px';
container.appendChild(progress);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '200px';
output.style.overflow = 'auto';
container.appendChild(output);

function addLog(message: string) {
  const logItem = document.createElement('div');
  logItem.style.fontSize = '12px';
  logItem.style.marginBottom = '2px';
  logItem.textContent = message;
  output.appendChild(logItem);
}

const totalItems = 10000;
const batchSize = 100;
const totalBatches = Math.ceil(totalItems / batchSize);
let processedBatches = 0;

addLog('Début du traitement...');
progress.textContent = 'Progression : 0%';

range(1, totalItems)
  .pipe(
    bufferCount(batchSize),
    observeOn(asapScheduler),  // Traitement asynchrone de chaque lot
    tap(batch => {
      // Simulation d'un calcul lourd
      const sum = batch.reduce((acc, n) => acc + n, 0);
      processedBatches++;
      const percent = Math.floor((processedBatches / totalBatches) * 100);
      progress.textContent = `Progression : ${percent}%`;

      if (processedBatches % 10 === 0 || processedBatches === totalBatches) {
        addLog(`Lot ${processedBatches}/${totalBatches} terminé (total : ${sum})`);
      }
    })
  )
  .subscribe({
    complete: () => {
      addLog('--- Tout le traitement est terminé ---');
      progress.textContent = 'Progression : 100% ✅';
    }
  });
```

- Traitement par lots de 10 000 éléments par groupes de 100
- Traitement avec `asapScheduler` sans bloquer l'UI
- Affichage de la progression en temps réel

## 🧪 Exemple de code pratique 2 : Optimisation des animations

Exemple d'animation fluide utilisant `animationFrameScheduler`.

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { observeOn, take, map } from 'rxjs';

// Création de l'interface utilisateur
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'observeOn - Animation';
container2.appendChild(title2);

const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = '#4CAF50';
box.style.position = 'relative';
box.style.transition = 'none';
container2.appendChild(box);

let position = 0;

interval(0)
  .pipe(
    observeOn(animationFrameScheduler),  // Exécution à 60fps
    take(180),  // 3 secondes (60fps × 3 secondes)
    map(() => {
      position += 2;  // Déplacement de 2px par frame
      return position;
    })
  )
  .subscribe({
    next: pos => {
      box.style.left = `${pos}px`;
    },
    complete: () => {
      const message = document.createElement('div');
      message.textContent = 'Animation terminée';
      message.style.marginTop = '10px';
      message.style.color = '#4CAF50';
      container2.appendChild(message);
    }
  });
```

- Synchronisation avec le cycle de rendu du navigateur via `animationFrameScheduler`
- Animation fluide à 60fps
- Pause automatique dans les onglets en arrière-plan

## 🆚 Différence avec subscribeOn

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, subscribeOn, tap } from 'rxjs';

console.log('=== observeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('Avant observeOn (synchrone)')),
    observeOn(asyncScheduler),
    tap(() => console.log('Après observeOn (asynchrone)'))
  )
  .subscribe();

console.log('=== subscribeOn ===');
of(1, 2, 3)
  .pipe(
    tap(() => console.log('Après subscribeOn (asynchrone)')),
    subscribeOn(asyncScheduler)
  )
  .subscribe();

// Sortie :
// === observeOn ===
// Avant observeOn (synchrone)
// Avant observeOn (synchrone)
// Avant observeOn (synchrone)
// === subscribeOn ===
// Après observeOn (asynchrone)
// Après observeOn (asynchrone)
// Après observeOn (asynchrone)
// Après subscribeOn (asynchrone)
// Après subscribeOn (asynchrone)
// Après subscribeOn (asynchrone)
```

| Opérateur | Portée | Contrôle du timing |
|:---|:---|:---|
| `observeOn` | Opérations suivantes uniquement | Timing d'émission des valeurs |
| `subscribeOn` | Flux entier | Timing de démarrage de l'abonnement |

> [!NOTE]
> Pour plus de détails sur `subscribeOn`, voir [subscribeOn](./subscribeOn).

## ⚠️ Notes importantes

### 1. La position de placement est importante

L'emplacement de `observeOn` détermine quelles opérations deviennent asynchrones.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn, map, tap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(() => console.log('Traitement 1 (synchrone)')),
    map(x => x * 2),
    observeOn(asyncScheduler),  // Asynchrone à partir d'ici
    tap(() => console.log('Traitement 2 (asynchrone)')),
    map(x => x + 10)
  )
  .subscribe();

// Traitement 1 est synchrone, Traitement 2 est asynchrone
```

### 2. Plusieurs observeOn ne s'accumulent pas

```ts
import { of, asyncScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    observeOn(queueScheduler)  // Le dernier planificateur est appliqué
  )
  .subscribe();
```

Le planificateur du dernier `observeOn` (`queueScheduler` dans ce cas) est utilisé.

### 3. Impact sur les performances

L'utilisation fréquente de `observeOn` peut causer une surcharge.

```ts
import { asyncScheduler, range, map, bufferCount, concatMap, from } from 'rxjs';
import { observeOn } from 'rxjs';

// ❌ Mauvais exemple : Rendre chaque valeur asynchrone
range(1, 1000)
  .pipe(
    map(x => x * 2),
    observeOn(asyncScheduler)  // 1000 setTimeout
  )
  .subscribe();

// ✅ Bon exemple : Traitement par lots
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10 setTimeout
    concatMap(batch => from(batch).pipe(map(x => x * 2)))
  )
  .subscribe();
```

## Comparaison des timings d'exécution

```ts
import { of, asyncScheduler, asapScheduler, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Début');

// Traitement synchrone
of('sync').subscribe(v => console.log(`2: ${v}`));

// queueScheduler
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(v => console.log(`3: ${v}`));

// asapScheduler
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(v => console.log(`4: ${v}`));

// asyncScheduler
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(v => console.log(`5: ${v}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Fin');

// Ordre d'exécution :
// 1: Début
// 2: sync
// 7: Fin
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```

## 📚 Opérateurs associés

- **[subscribeOn](./subscribeOn)** - Contrôle du timing de démarrage de l'abonnement
- **[delay](./delay)** - Délai à temps fixe
- **[debounceTime](../filtering/debounceTime)** - Délai après arrêt de l'entrée

## 📖 Documentation associée

- **[Contrôle du traitement asynchrone](/fr/guide/schedulers/async-control)** - Bases des planificateurs
- **[Types et utilisation des planificateurs](/fr/guide/schedulers/types)** - Détails de chaque planificateur

## ✅ Résumé

L'opérateur `observeOn` contrôle le timing d'émission des valeurs et le contexte d'exécution.

- ✅ Exécution des opérations suivantes sur le planificateur spécifié
- ✅ Efficace pour éviter le blocage de l'UI
- ✅ Utile pour l'optimisation des animations
- ✅ Permet la priorisation du traitement
- ⚠️ La position de placement est importante
- ⚠️ Attention à la surcharge de performance
- ⚠️ En cas d'utilisation multiple, le dernier planificateur est appliqué
