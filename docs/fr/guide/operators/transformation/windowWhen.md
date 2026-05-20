---
description: "L'opérateur windowWhen scinde un Observable en contrôlant dynamiquement la condition de fin. Comme la fenêtre suivante commence immédiatement après la fin de la fenêtre, il est idéal pour le fractionnement continu des données ; la différence avec bufferWhen, son implémentation à sécurité de type dans TypeScript et des exemples pratiques seront expliqués."
---

# windowWhen - fenêtre de contrôle de fin dynamique

L'opérateur `windowWhen` divise un Observable en **contrôlant dynamiquement la condition de sortie**. Il permet un modèle de traitement de flux continu, où la fenêtre suivante commence immédiatement lorsqu'une fenêtre se termine.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500); // 0.5Valeur d'émission toutes les secondes

// Condition de fin: 1Après chaque seconde
const closingSelector = () => interval(1000);

source$.pipe(
  windowWhen(closingSelector),
  take(4),
  mergeAll()
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
});

// Fenêtre1: 0       (après0Début des secondes → 1(Fin des secondes)
// Fenêtre2: 1, 2    (après1Début des secondes → 2(Fin des secondes)
// Fenêtre3: 3, 4    (après2Début des secondes → 3(Fin des secondes)
// Fenêtre4: 5, 6    (après3Début des secondes → 4(Fin des secondes)
```

**Flux d'opérations** :.
1. la première fenêtre démarre automatiquement
2. l'Observable renvoyé par `closingSelector()` émet une valeur → la fenêtre se termine
3. **immédiatement la fenêtre suivante démarre** 4.
4. 2-3 répétées

[🌐 Official RxJS documentation - `windowWhen`](https://rxjs.dev/api/operators/windowWhen)

## 💡 Modèle d'utilisation typique

- Collecte de données dans des intervalles de temps dynamiques
- Traitement adaptatif des flux en fonction de la charge
- Contrôle des fenêtres en fonction des résultats précédents
- Regroupement continu des données

## 🔍 Différences avec bufferWhen

TABLEAU_10___.

```ts
import { interval } from 'rxjs';
import { bufferWhen, windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500);
const closing = () => interval(1000);

// bufferWhen - Sortie sous forme de tableau
source$.pipe(
  bufferWhen(closing),
  take(3)
).subscribe(values => {
  console.log('Buffer (tableau):', values);
  // Sortie en tant que: Buffer (tableau): [0]
  // Sortie en tant que: Buffer (tableau): [1, 2]
  // Sortie en tant que: Buffer (tableau): [3, 4]
});

// windowWhen - Observable Sortie en tant que
source$.pipe(
  windowWhen(closing),
  take(3),
  mergeAll()
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
  // Sortie en tant que: Valeur dans la fenêtre: 0
  // Sortie en tant que: Valeur dans la fenêtre: 1
  // Sortie en tant que: Valeur dans la fenêtre: 2
  // ...
});
```

## 🧠 Exemple de code pratique 1 : Collecte de données à intervalles de temps dynamiques

Voici un exemple d'ajustement de la période de fenêtre suivante en fonction des résultats de la fenêtre précédente.

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, scan, map } from 'rxjs';

// Données du capteur (générées en permanence)
const sensorData$ = interval(100).pipe(
  map(() => ({
    timestamp: Date.now(),
    temperature: 20 + Math.random() * 10 // 20-30Degré
  }))
);

let windowNumber = 0;
let previousAvgTemp = 25;

sensorData$.pipe(
  windowWhen(() => {
    const current = ++windowNumber;
    // Plus la température est élevée, plus l'intervalle est court Fenêtre
    const duration = previousAvgTemp > 27 ? 500 : 1000;
    console.log(`Fenêtre ${current} Début (durée: ${duration}ms)`);
    return timer(duration);
  }),
  mergeMap(window$ => {
    const currentWindow = windowNumber;  // Conserver le numéro de la fenêtre actuelle
    return window$.pipe(
      toArray(),
      map(data => {
        const avgTemp = data.reduce((sum, d) => sum + d.temperature, 0) / data.length;
        previousAvgTemp = avgTemp;
        return {
          window: currentWindow,  // Utiliser le numéro de fenêtre retenu
          count: data.length,
          avgTemp
        };
      })
    );
  })
).subscribe(stats => {
  console.log(`Fenêtre ${stats.window}: Température moyenne ${stats.avgTemp.toFixed(1)}°C, Nombre d'échantillons ${stats.count}Nombre de cas`);
});
```

## 🎯 Exemple de code pratique 2 : traitement adaptatif des flux en fonction de la charge

Voici un exemple de modification dynamique de la longueur de la fenêtre en fonction de la charge du système.

```ts
import { interval, timer, fromEvent } from 'rxjs';
import { windowWhen, mergeMap, scan, map } from 'rxjs';

// Créer une zone de sortie
const container = document.createElement('div');
document.body.appendChild(container);

const loadButton = document.createElement('button');
loadButton.textContent = 'Générer une charge';
container.appendChild(loadButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'Faibles charges: 5Collecte à intervalles d'une seconde';
container.appendChild(status);

const logDisplay = document.createElement('div');
logDisplay.style.marginTop = '10px';
logDisplay.style.maxHeight = '300px';
logDisplay.style.overflow = 'auto';
container.appendChild(logDisplay);

// Flux de données (généré en permanence)
let logCounter = 0;
const logs$ = interval(200).pipe(
  map(() => ({
    id: logCounter++,
    level: Math.random() > 0.7 ? 'ERROR' : 'INFO',
    timestamp: new Date()
  }))
);

// Niveau de charge
let loadLevel = 0;
fromEvent(loadButton, 'click').subscribe(() => {
  loadLevel = Math.min(loadLevel + 1, 5);
  updateStatus();
});

// 30Charge décroissante chaque seconde
interval(30000).subscribe(() => {
  loadLevel = Math.max(loadLevel - 1, 0);
  updateStatus();
});

function updateStatus() {
  const interval = getWindowDuration(loadLevel);
  const loadText = loadLevel === 0 ? 'Faibles charges' :
                   loadLevel <= 2 ? 'Charge moyenne' : 'Charge élevée';
  status.textContent = `${loadText} (Level ${loadLevel}): ${interval / 1000}Collecte à intervalles d'une seconde`;
}

function getWindowDuration(load: number): number {
  // Intervalles plus courts pour les charges élevées
  switch (load) {
    case 0: return 5000;
    case 1: return 3000;
    case 2: return 2000;
    case 3: return 1000;
    case 4: return 500;
    default: return 300;
  }
}

let windowNum = 0;

// Fenêtrage adaptatif
logs$.pipe(
  windowWhen(() => {
    windowNum++;
    return timer(getWindowDuration(loadLevel));
  }),
  mergeMap(window$ =>
    window$.pipe(
      scan((stats, log) => ({
        count: stats.count + 1,
        errors: stats.errors + (log.level === 'ERROR' ? 1 : 0),
        window: windowNum
      }), { count: 0, errors: 0, window: windowNum })
    )
  )
).subscribe(stats => {
  const timestamp = new Date().toLocaleTimeString();
  const div = document.createElement('div');
  div.textContent = `[${timestamp}] Fenêtre ${stats.window}: ${stats.count}Nombre de cas (Erreur: ${stats.errors}Nombre de cas)`;
  logDisplay.insertBefore(div, logDisplay.firstChild);
});
```

## 🆚 Différences avec windowToggle

```ts
import { interval, timer } from 'rxjs';
import { windowWhen, windowToggle, mergeAll } from 'rxjs';

const source$ = interval(200);

// windowWhen: Contrôle de fin uniquement (le prochain démarrage a lieu immédiatement après la fin)
source$.pipe(
  windowWhen(() => timer(1000)),
  mergeAll()
).subscribe();

// windowToggle: Contrôle séparé du début et de la fin々Contrôle séparé du début et de la fin
source$.pipe(
  windowToggle(
    interval(1000),          // Déclenchement du début
    () => timer(500)         // Déclenchement de la fin (après le début)500msAprès)
  ),
  mergeAll()
).subscribe();
```

```ts
import { interval } from 'rxjs';
import { windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500); // 0.5Valeur d'émission toutes les secondes

// Condition de fin: 1Après chaque seconde
const closingSelector = () => interval(1000);

source$.pipe(
  windowWhen(closingSelector),
  take(4),
  mergeAll()
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
});

// Fenêtre1: 0       (après0Début des secondes → 1(Fin des secondes)
// Fenêtre2: 1, 2    (après1Début des secondes → 2(Fin des secondes)
// Fenêtre3: 3, 4    (après2Début des secondes → 3(Fin des secondes)
// Fenêtre4: 5, 6    (après3Début des secondes → 4(Fin des secondes)
```

**Points d'utilisation** :.
- **`fenêtreWhen`** : pour traiter toutes les données en continu et sans omissions (journalisation, agrégation de données, etc.).
- **`windowToggle`** : pour traiter les données uniquement pendant une période de temps spécifique (par exemple, pendant les heures de bureau, lors de l'appui sur un bouton).

## 🎯 Exemple pratique : contrôle adaptatif de la taille de la fenêtre

Voici un exemple d'ajustement automatique de la période de fenêtre suivante en fonction des résultats de la fenêtre précédente.


```ts
import { interval, timer } from 'rxjs';
import { windowWhen, mergeMap, toArray, map } from 'rxjs';

interface WindowStats {
  count: number;
  nextDuration: number;
}

const data$ = interval(100);

let previousCount = 0;

// Période de la fenêtre suivante ajustée en fonction du volume de données
function getNextDuration(count: number): number {
  if (count > 20) {
    return 500;  // Volume de données élevé → Intervalle court
  } else if (count > 10) {
    return 1000; // Intervalle moyen → Intervalle intermédiaire
  } else {
    return 2000; // Faible volume de données → Intervalle long
  }
}

data$.pipe(
  windowWhen(() => timer(getNextDuration(previousCount))),
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(data => {
        previousCount = data.length;
        return {
          count: data.length,
          nextDuration: getNextDuration(data.length)
        } as WindowStats;
      })
    )
  )
).subscribe(stats => {
  console.log(`Taille de la fenêtre: ${stats.count}Nombre de cas, Période suivante: ${stats.nextDuration}ms`);
});
```

## ⚠️ Notes.

### 1. gestion des abonnements aux fenêtres

Chaque fenêtre est un Observable indépendant et doit être explicitement souscrite ou aplatie avec `mergeAll()` etc.

```ts
source$.pipe(
  windowWhen(closing)
).subscribe(window$ => {
  // Les valeurs ne circulent pas à moins que la fenêtre elle-même ne soit souscrite.
  window$.subscribe(value => {
    console.log('Valeur:', value);
  });
});
```

### 2. un nouvel Observable doit être retourné à chaque fois.

La fonction `closingSelector` **doit retourner un nouvel Observable à chaque fois**. Si elle retourne la même instance, elle ne fonctionnera pas correctement.

```ts
// ❌ Mauvais exemple: Si la mêmeObservableest utilisée partout.
const closingObservable = timer(1000);

source$.pipe(
  windowWhen(() => closingObservable) // 2Ne fonctionne pas après la deuxième fois！
).subscribe();

// ✅ Bon exemple: Une nouvelleObservableà chaque fois
source$.pipe(
  windowWhen(() => timer(1000)) // Une nouvelletimerà chaque fois
).subscribe();
```

### Notez la complexité des conditions de fermeture.

Si les conditions de sortie deviennent trop complexes, le débogage devient difficile.

```ts
// Exemple trop complexe
let counter = 0;
source$.pipe(
  windowWhen(() => {
    counter++;
    const duration = counter % 3 === 0 ? 500 :
                     counter % 2 === 0 ? 1000 : 1500;
    return timer(duration);
  })
).subscribe();
// Difficile à déboguer
```

## 🆚 Comparaison des opérateurs basés sur des fenêtres

```ts
import { interval } from 'rxjs';
import { windowWhen, mergeAll, take } from 'rxjs';

const source$ = interval(500); // 0.5Valeur d'émission toutes les secondes

// Condition de fin: 1Après chaque seconde
const closingSelector = () => interval(1000);

source$.pipe(
  windowWhen(closingSelector),
  take(4),
  mergeAll()
).subscribe(value => {
  console.log('Valeur dans la fenêtre:', value);
});

// Fenêtre1: 0       (après0Début des secondes → 1(Fin des secondes)
// Fenêtre2: 1, 2    (après1Début des secondes → 2(Fin des secondes)
// Fenêtre3: 3, 4    (après2Début des secondes → 3(Fin des secondes)
// Fenêtre4: 5, 6    (après3Début des secondes → 4(Fin des secondes)
```

## 📚 Opérateurs apparentés.

- [`bufferWhen`](./bufferWhen) - résume les valeurs sous forme de tableau (version tableau de windowWhen).
- [`window`](./window) - fractionnement de la fenêtre à différents moments de l'Observable.
- [`windowTime`](./windowTime) - fractionnement de la fenêtre en fonction du temps.
- [`windowCount`](./windowCount) - fractionnement des fenêtres basé sur la quantité.
- [`windowToggle`](./windowToggle) - contrôle de la fenêtre avec début et fin Observable

## Résumé.

L'opérateur `windowWhen` est un outil utile pour contrôler dynamiquement les conditions de fin et le fenêtrage continu.

- ✅ Les conditions de sortie peuvent être contrôlées dynamiquement
- ✅ Traitement continu des fenêtres (pas de fuite de données)
- La fenêtre suivante peut être ajustée en fonction des résultats précédents.
- ⚠️ Gestion de l'abonnement nécessaire
- ⚠️ Nécessité de renvoyer un nouvel Observable à chaque fois
- ⚠️ Attention à ne pas trop compliquer les conditions de sortie
