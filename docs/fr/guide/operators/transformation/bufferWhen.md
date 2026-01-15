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

### Points d'attention
- ⚠️ `closingSelector` doit retourner un nouvel Observable à chaque fois
- ⚠️ Si les conditions de fin deviennent trop complexes, le débogage devient difficile


## 🚀 Prochaines étapes

- **[buffer](./buffer)** - Apprendre la mise en buffer de base
- **[bufferTime](./bufferTime)** - Apprendre la mise en buffer basée sur le temps
- **[bufferCount](./bufferCount)** - Apprendre la mise en buffer basée sur le nombre
- **[bufferToggle](./bufferToggle)** - Apprendre la mise en buffer avec contrôle indépendant début/fin
- **[Exemples pratiques d'opérateurs de transformation](./practical-use-cases)** - Apprendre les cas d'utilisation réels
