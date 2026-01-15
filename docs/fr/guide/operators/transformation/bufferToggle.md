---
description: "L'opérateur bufferToggle contrôle les déclencheurs de début et de fin avec des Observables séparés, permettant de gérer indépendamment plusieurs périodes de mise en buffer. Un opérateur avancé de mise en buffer."
---

# bufferToggle - Buffer avec déclencheurs

L'opérateur `bufferToggle` contrôle les **déclencheurs de début** et **de fin** avec des Observables séparés et émet les valeurs regroupées dans un tableau. C'est un opérateur de mise en buffer avancé capable de gérer plusieurs périodes de mise en buffer simultanément.


## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { bufferToggle } from 'rxjs';

const source$ = interval(500); // Émet toutes les 0.5 secondes

// Déclencheur de début : toutes les 2 secondes
const opening$ = interval(2000);

// Déclencheur de fin : 1 seconde après le début
const closing = () => interval(1000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Sortie :
// [3, 4, 5]     (début à 2s, fin à 3s)
// [7, 8, 9]     (début à 4s, fin à 5s)
// [11, 12, 13]  (début à 6s, fin à 7s)
```

**Flux d'opération** :
1. `opening$` émet une valeur → début de la mise en buffer
2. L'Observable retourné par `closing()` émet une valeur → fin de la mise en buffer, sortie du tableau
3. Plusieurs périodes de mise en buffer peuvent se chevaucher

[🌐 Documentation officielle RxJS - `bufferToggle`](https://rxjs.dev/api/operators/bufferToggle)


## 🆚 Comparaison avec d'autres opérateurs buffer

| Opérateur | Déclencheur | Caractéristique | Cas d'utilisation |
|---|---|---|---|
| `buffer(trigger$)` | Observable unique | Simple | Mise en buffer pilotée par événements |
| `bufferTime(ms)` | Temps | Périodique | Agrégation à intervalle fixe |
| `bufferCount(n)` | Nombre | Quantitatif | Traitement par N éléments |
| `bufferToggle(open$, close)` | Début et fin séparés | Flexible | Gestion de périodes complexes |


## 💡 Patterns d'utilisation typiques

1. **Collecte de données pendant les heures de bureau**
2. **Enregistrement d'événements pendant l'appui sur un bouton**
3. **Enregistrement des actions des utilisateurs actifs**


## 🧠 Exemple de code pratique (gestion de période de téléchargement)

Exemple de gestion des périodes de téléchargement de données avec des boutons de démarrage et d'arrêt.

```ts
import { interval, fromEvent, Subject } from 'rxjs';
import { bufferToggle, map, take } from 'rxjs';

// Création des éléments UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Gestion de téléchargement de données';
container.appendChild(title);

const startButton = document.createElement('button');
startButton.textContent = 'Démarrer';
container.appendChild(startButton);

const stopButton = document.createElement('button');
stopButton.textContent = 'Arrêter';
stopButton.disabled = true;
stopButton.style.marginLeft = '10px';
container.appendChild(stopButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = 'En attente...';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
container.appendChild(result);

// Flux de données (génère des données de téléchargement toutes les secondes)
const downloadData$ = interval(1000).pipe(
  map(i => ({
    id: i,
    size: Math.floor(Math.random() * 1000) + 100, // 100-1100KB
    timestamp: new Date()
  }))
);

// Déclencheurs de début et de fin
const start$ = fromEvent(startButton, 'click');
const stop$ = new Subject<void>();

fromEvent(stopButton, 'click').subscribe(() => {
  stop$.next();
  status.textContent = 'Arrêté';
  startButton.disabled = false;
  stopButton.disabled = true;
});

start$.subscribe(() => {
  status.textContent = 'Téléchargement en cours...';
  startButton.disabled = true;
  stopButton.disabled = false;
});

// Mise en buffer
downloadData$.pipe(
  bufferToggle(start$, () => stop$)
).subscribe(downloads => {
  const totalSize = downloads.reduce((sum, d) => sum + d.size, 0);
  const avgSize = downloads.length > 0 ? totalSize / downloads.length : 0;

  result.innerHTML = `
    <strong>Téléchargement terminé</strong><br>
    Nombre : ${downloads.length} fichiers<br>
    Taille totale : ${(totalSize / 1024).toFixed(2)} MB<br>
    Taille moyenne : ${avgSize.toFixed(0)} KB
  `;
});
```


## 🎯 Périodes de buffer qui se chevauchent

Une caractéristique de `bufferToggle` est qu'il peut gérer plusieurs périodes de mise en buffer simultanément.

```ts
import { interval } from 'rxjs';
import { bufferToggle, take } from 'rxjs';

const source$ = interval(200).pipe(take(20)); // 0-19

// Début : toutes les secondes
const opening$ = interval(1000);

// Fin : 1.5 secondes après le début
const closing = () => interval(1500);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Sortie :
// [4, 5, 6]        (début 1s → fin 2.5s)
// [9, 10, 11, 12]  (début 2s → fin 3.5s) ※ chevauchement partiel
// [14, 15, 16, 17] (début 3s → fin 4.5s)
```


## ⚠️ Erreurs courantes

> [!WARNING]
> `bufferToggle` peut gérer plusieurs périodes de buffer simultanément, mais si le déclencheur de début se déclenche fréquemment, de nombreux buffers existeront en même temps, consommant de la mémoire.

### Incorrect : déclencheur de début trop fréquent

```ts
// ❌ Mauvais exemple : début toutes les 100ms, fin 5s plus tard
const opening$ = interval(100); // Trop fréquent
const closing = () => interval(5000);

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Jusqu'à 50 buffers peuvent exister simultanément → risque mémoire
```

### Correct : intervalle approprié

```ts
// ✅ Bon exemple : intervalles appropriés
const opening$ = interval(2000); // Toutes les 2 secondes
const closing = () => interval(1000); // Buffer pendant 1 seconde

source$.pipe(
  bufferToggle(opening$, closing)
).subscribe(console.log);
// Maximum 1-2 buffers simultanés
```


## 🎓 Résumé

### Quand utiliser bufferToggle
- ✅ Lorsque vous voulez contrôler le début et la fin indépendamment
- ✅ Lorsque vous voulez collecter des données pendant une période limitée (appui sur bouton, etc.)
- ✅ Lorsque vous voulez gérer plusieurs périodes de buffer simultanément
- ✅ Collecte de données avec conditions complexes (heures de bureau uniquement, etc.)

### Points d'attention
- ⚠️ Si le déclencheur de début est fréquent, de nombreux buffers existeront simultanément, consommant de la mémoire
- ⚠️ Les périodes de buffer peuvent se chevaucher
- ⚠️ Le contrôle complexe peut rendre le débogage difficile


## 🚀 Prochaines étapes

- **[buffer](./buffer)** - Apprendre la mise en buffer de base
- **[bufferTime](./bufferTime)** - Apprendre la mise en buffer basée sur le temps
- **[bufferCount](./bufferCount)** - Apprendre la mise en buffer basée sur le nombre
- **[bufferWhen](./bufferWhen)** - Apprendre le contrôle dynamique de fin
- **[Exemples pratiques d'opérateurs de transformation](./practical-use-cases)** - Apprendre les cas d'utilisation réels
