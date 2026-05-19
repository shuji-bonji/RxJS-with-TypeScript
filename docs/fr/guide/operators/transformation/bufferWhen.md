---
description: "bufferWhen est un opérateur de transformation RxJS qui contrôle dynamiquement les conditions de fin pour regrouper et émettre des valeurs sous forme de tableau. Il réalise une mise en buffer continue où le buffer suivant démarre immédiatement après la fin du précédent, utile pour le traitement par lots adaptatif et la collecte de logs selon la charge. L'inférence de type TypeScript permet une mise en buffer dynamique type-safe."
---

# bufferWhen - Buffer dynamique

L'opérateur `bufferWhen` **contrôle dynamiquement les conditions de fin** et publie les valeurs dans un tableau. Il fournit un pattern de mise en buffer continue, où lorsqu'un buffer se termine, le buffer suivant démarre immédiatement.


## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(500); // Émet toutes les 0.5 secondes

// Condition de fin : après 1 seconde
const closingSelector = () => interval(1000);

source$.pipe(
  bufferWhen(closingSelector),
  take(4)
).subscribe(console.log);
// Sortie :
// [0]           (démarrage 0s → fin 1s, valeur 0 uniquement)
// [1, 2, 3]     (démarrage 1s → fin 2s, valeurs 1,2,3)
// [4, 5]        (démarrage 2s → fin 3s, valeurs 4,5)
// [6, 7]        (démarrage 3s → fin 4s, valeurs 6,7)
```

**Flux d'opération** :
1. Le premier buffer démarre automatiquement
2. L'Observable retourné par `closingSelector()` émet une valeur → fin du buffer, sortie du tableau
3. **Immédiatement, le buffer suivant démarre**
4. Répétition de 2-3

> [!NOTE]
> Le premier buffer dure 1 seconde (jusqu'à ce que `interval(1000)` émette sa première valeur), donc seulement `[0]`. À partir du deuxième buffer, le démarrage et l'émission de `source$` coïncident souvent, incluant plus de valeurs.

[🌐 Documentation officielle RxJS - `bufferWhen`](https://rxjs.dev/api/operators/bufferWhen)


## 🆚 Différences avec bufferToggle

`bufferWhen` et `bufferToggle` sont similaires, mais **leurs méthodes de contrôle et comportements sont très différents**.

### Comportement de bufferWhen

```ts
import { interval } from 'rxjs';
import { bufferWhen, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // Émet 0-11 toutes les 300ms

// bufferWhen : contrôle uniquement la fin (le buffer suivant démarre immédiatement)
source$.pipe(
  bufferWhen(() => interval(1000))
).subscribe(console.log);
// Sortie : [0, 1, 2], [3, 4, 5], [6, 7, 8, 9], [10, 11]
//
// Chronologie :
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms 3600ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//  [----------1s----------][----------1s----------][----------1s----------][-----1s-----]
//   Buffer 1 (0-2)          Buffer 2 (3-5)          Buffer 3 (6-9)          Buffer 4 (10-11)
//   Continu, sans chevauchement, le buffer suivant démarre immédiatement
```

### Comportement de bufferToggle

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(12)); // Émet 0-11 toutes les 300ms

// bufferToggle : contrôle indépendant du début et de la fin (chevauchement possible)
const opening$ = interval(1000); // Démarrage toutes les 1s
const closing = () => interval(800); // Fin 800ms après le démarrage

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Sortie : [3, 4, 5], [6, 7, 8], [9, 10, 11]
//
// Chronologie :
//  0ms   300ms  600ms  900ms  1200ms 1500ms 1800ms 2100ms 2400ms 2700ms 3000ms 3300ms
//  0     1      2      3      4      5      6      7      8      9      10     11
//        ----Début 1 (1000ms)----[---fin 800ms plus tard (1800ms)---]
//                        3      4      5
//                        └→ [3,4,5]
//                    ----Début 2 (2000ms)----[---fin 800ms plus tard (2800ms)---]
//                                            6      7      8
//                                            └→ [6,7,8]
//                              ----Début 3 (3000ms)----[---fin 800ms plus tard (3800ms)---]
//                                                      9      10     11
//                                                      └→ [9,10,11]
//  Attend le déclencheur de démarrage, périodes indépendantes (0-2 ne sont pas inclus car avant le premier démarrage)
```

### Principales différences

| Opérateur | Contrôle de début | Contrôle de fin | Période de buffer | Caractéristique |
|---|---|---|---|---|
| `bufferWhen(closing)` | Auto (immédiat après fin) | Dynamique | Continue | Pas d'intervalle entre buffers |
| `bufferToggle(open$, close)` | Observable indépendant | Dynamique | Indépendante, chevauchement possible | Intervalles entre buffers |

**Points de différenciation** :
- **`bufferWhen`** : pour mettre en buffer toutes les données en continu sans omissions (collecte de logs, agrégation de données, etc.)
- **`bufferToggle`** : pour collecter des données pendant une période spécifique (heures de bureau, pendant l'appui sur un bouton, etc.)

> [!TIP]
> - **Mise en buffer continue** (ne pas perdre de données) → `bufferWhen`
> - **Mise en buffer limitée dans le temps** (contrôle explicite début/fin) → `bufferToggle`


## 💡 Patterns d'utilisation typiques

1. **Collecte de données à intervalles dynamiques**
2. **Traitement par lots adaptatif selon la charge**
3. **Échantillonnage à intervalles aléatoires**


## 🧠 Exemple de code pratique (collecte de logs selon la charge)

Exemple de modification dynamique de la fréquence de collecte des logs selon la charge du système.

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { bufferWhen, map, share } from 'rxjs';

// Création des éléments UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Système de collecte de logs adaptatif';
container.appendChild(title);

const loadButton = document.createElement('button');
loadButton.textContent = 'Générer de la charge';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.style.padding = '10px';
status.style.backgroundColor = '#f0f0f0';
status.textContent = 'Charge faible : collecte toutes les 5 secondes';
container.appendChild(status);

// Flux de logs (génération continue)
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    message: `Log message ${logCounter}`,
    timestamp: new Date()
  })),
  share()
);

// Compteur de charge (incrémenté au clic du bouton)
let loadLevel = 0;
fromEvent(loadButton, 'click').subscribe(() => {
  loadLevel = Math.min(loadLevel + 1, 5);
  updateStatus();
});

function updateStatus() {
  const intervalMs = getBufferInterval(loadLevel);
  const loadText = loadLevel === 0 ? 'Charge faible' :
                   loadLevel <= 2 ? 'Charge moyenne' : 'Charge élevée';
  status.textContent = `${loadText} (Niveau ${loadLevel}) : collecte toutes les ${intervalMs / 1000}s`;
}

function getBufferInterval(load: number): number {
  // Plus la charge est élevée, plus l'intervalle est court
  switch (load) {
    case 0: return 5000;  // 5s
    case 1: return 3000;  // 3s
    case 2: return 2000;  // 2s
    case 3: return 1000;  // 1s
    case 4: return 500;   // 0.5s
    default: return 300;  // 0.3s
  }
}

// Mise en buffer adaptative
logs$.pipe(
  bufferWhen(() => timer(getBufferInterval(loadLevel)))
).subscribe(bufferedLogs => {
  if (bufferedLogs.length > 0) {
    const errors = bufferedLogs.filter(log => log.level === 'ERROR').length;
    console.log(`Collecté : ${bufferedLogs.length} logs (erreurs : ${errors})`);
  }
});
```


## 📋 Utilisation type-safe

Un exemple d'implémentation type-safe utilisant les génériques en TypeScript.

```ts
import { Observable, interval, timer } from 'rxjs';
import { bufferWhen, map } from 'rxjs';

interface MetricData {
  value: number;
  timestamp: Date;
  source: string;
}

interface BufferConfig {
  minDuration: number;
  maxDuration: number;
  adaptive: boolean;
}

class AdaptiveBuffer<T> {
  constructor(private config: BufferConfig) {}

  private getNextBufferDuration(previousCount: number): number {
    if (!this.config.adaptive) {
      return this.config.minDuration;
    }

    // Ajuste la prochaine période de buffer en fonction du volume de données
    const ratio = Math.min(previousCount / 10, 1);
    const duration =
      this.config.minDuration +
      (this.config.maxDuration - this.config.minDuration) * (1 - ratio);

    return Math.floor(duration);
  }

  apply(source$: Observable<T>): Observable<T[]> {
    let previousCount = 0;

    return source$.pipe(
      bufferWhen(() => {
        const duration = this.getNextBufferDuration(previousCount);
        return timer(duration);
      }),
      map(buffer => {
        previousCount = buffer.length;
        return buffer;
      })
    );
  }
}

// Exemple d'utilisation
const metricsStream$ = interval(300).pipe(
  map(i => ({
    value: Math.random() * 100,
    timestamp: new Date(),
    source: `sensor-${i % 3}`
  } as MetricData))
);

const buffer = new AdaptiveBuffer<MetricData>({
  minDuration: 1000,  // minimum 1s
  maxDuration: 5000,  // maximum 5s
  adaptive: true      // adaptatif
});

buffer.apply(metricsStream$).subscribe(metrics => {
  if (metrics.length > 0) {
    const avg = metrics.reduce((sum, m) => sum + m.value, 0) / metrics.length;
    console.log(`Taille du buffer : ${metrics.length}, moyenne : ${avg.toFixed(2)}`);
  }
});
```


## 🎯 Comparaison avec les autres opérateurs de buffer

```ts
import { interval, timer, Subject } from 'rxjs';
import { buffer, bufferTime, bufferCount, bufferWhen, bufferToggle, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0-9

// 1. buffer : déclencheur fixe
const trigger$ = new Subject<void>();
source$.pipe(buffer(trigger$)).subscribe(console.log);
setInterval(() => trigger$.next(), 1000);
// Sortie : [0, 1, 2], [3, 4, 5], ... (au moment du déclencheur)

// 2. bufferTime : intervalle de temps fixe
source$.pipe(bufferTime(1000)).subscribe(console.log);
// Sortie : [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 3. bufferCount : nombre fixe
source$.pipe(bufferCount(3)).subscribe(console.log);
// Sortie : [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 4. bufferWhen : contrôle dynamique de fin (continu)
source$.pipe(
  bufferWhen(() => timer(1000))
).subscribe(console.log);
// Sortie : [0, 1, 2], [3, 4, 5], [6, 7, 8], [9]

// 5. bufferToggle : contrôle indépendant du début et de la fin (chevauchement possible)
const opening$ = interval(1000);
const closing = () => timer(800);
source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Sortie : [3, 4, 5], [6, 7, 8]
```

| Opérateur | Déclencheur | Contrôle dynamique | Chevauchement | Cas d'utilisation |
|---|---|---|---|---|
| `buffer` | Observable externe | ❌ | ❌ | Événementiel |
| `bufferTime` | Temps fixe | ❌ | ❌ | Agrégation périodique |
| `bufferCount` | Nombre fixe | ❌ | ❌ | Traitement quantitatif |
| `bufferWhen` | Dynamique (fin uniquement) | ✅ | ❌ | Traitement par lots adaptatif |
| `bufferToggle` | Dynamique (début et fin) | ✅ | ✅ | Gestion de périodes complexes |


## ⚠️ Erreurs courantes

> [!WARNING]
> La fonction de condition de fin de `bufferWhen` doit **retourner un nouvel Observable à chaque fois**. Retourner la même instance d'Observable ne fonctionnera pas correctement.

### Incorrect : retourner la même instance d'Observable

```ts
const source$ = interval(500);

// ❌ Mauvais exemple : réutilisation de la même instance Observable
const closingObservable = timer(1000);

source$.pipe(
  bufferWhen(() => closingObservable) // Ne fonctionne pas après le premier buffer !
).subscribe(console.log);
```

### Correct : retourner un nouvel Observable à chaque fois

```ts
const source$ = interval(500);

// ✅ Bon exemple : génère un nouvel Observable à chaque fois
source$.pipe(
  bufferWhen(() => timer(1000)) // Génère un nouveau timer à chaque fois
).subscribe(console.log);
// Sortie : [0, 1], [2, 3], [4, 5], ...
```


## 🎓 Résumé

### Quand utiliser bufferWhen
- ✅ Lorsque vous voulez contrôler dynamiquement les conditions de fin
- ✅ Lorsque vous avez besoin de périodes de buffer continues
- ✅ Lorsque vous voulez ajuster la prochaine période basée sur les résultats précédents
- ✅ Lorsque vous voulez implémenter un traitement par lots adaptatif

### Quand utiliser bufferToggle
- ✅ Lorsque vous voulez contrôler le début et la fin de manière indépendante
- ✅ Lorsque les périodes de buffer peuvent se chevaucher
- ✅ Lorsque vous avez des événements de début/fin clairs (ex. pendant l'appui sur un bouton)

### Quand utiliser bufferTime
- ✅ Lorsque la mise en buffer à intervalles de temps fixes est suffisante
- ✅ Lorsqu'une implémentation simple est requise

### Points d'attention
- ⚠️ `closingSelector` doit retourner un nouvel Observable à chaque fois
- ⚠️ Des conditions de fin trop complexes rendent le débogage difficile
- ⚠️ Pour le contrôle adaptatif, les tests sont importants pour éviter les comportements inattendus


## 🚀 Prochaines étapes

- **[buffer](./buffer)** - Apprendre la mise en buffer de base
- **[bufferTime](./bufferTime)** - Apprendre la mise en buffer basée sur le temps
- **[bufferCount](./bufferCount)** - Apprendre la mise en buffer basée sur le nombre
- **[bufferToggle](./bufferToggle)** - Apprendre la mise en buffer avec contrôle indépendant début/fin
- **[Exemples pratiques d'opérateurs de transformation](./practical-use-cases)** - Apprendre les cas d'utilisation réels
