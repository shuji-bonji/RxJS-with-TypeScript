---
description: "L'opérateur sampleTime est un opérateur de filtrage RxJS qui échantillonne périodiquement la dernière valeur du flux à des intervalles spécifiés. Idéal pour obtenir des snapshots périodiques."
---

# sampleTime - Échantillonnage périodique

L'opérateur `sampleTime` **échantillonne périodiquement** la **dernière valeur** de l'Observable source à intervalles réguliers.
Comme un snapshot périodique, il récupère la dernière valeur à ce moment.

## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Échantillon toutes les 2 secondes');
});
```

**Flux d'opération** :
1. Un timer se déclenche périodiquement toutes les 2 secondes
2. S'il y a un événement de clic récent à ce moment, il est émis
3. Si aucune valeur n'existe pendant la période d'échantillonnage, rien n'est émis

[🌐 Documentation officielle RxJS - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

> [!WARNING] Attention en code de production
> L'exemple ci-dessus omet la désinscription de `fromEvent` pour simplifier l'explication. Dans du code réel, gérez explicitement le cycle de vie avec `takeUntil(destroy$)`, `take(N)`, ou `Subscription.unsubscribe()`. Détails : [Surmonter les difficultés : gestion du cycle de vie](/fr/guide/overcoming-difficulties/lifecycle-management.md)

## 💡 Patterns d'utilisation typiques

- **Récupération périodique de données de capteur** : Température ou position la plus récente chaque seconde
- **Tableau de bord temps réel** : Mise à jour périodique de l'état
- **Surveillance des performances** : Collecte de métriques à intervalles réguliers
- **Traitement de frames de jeu** : Échantillonnage périodique pour le contrôle FPS

## 🧠 Exemple de code pratique 1 : Échantillonnage périodique de position de souris

Un exemple d'échantillonnage et d'affichage de la position de la souris chaque seconde.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// Création de l'UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Échantillonnage de position de souris (chaque seconde)';
container.appendChild(title);

const area = document.createElement('div');
area.style.width = '100%';
area.style.height = '300px';
area.style.border = '2px solid #4CAF50';
area.style.backgroundColor = '#f5f5f5';
area.style.display = 'flex';
area.style.alignItems = 'center';
area.style.justifyContent = 'center';
area.style.fontSize = '18px';
area.textContent = 'Déplacez la souris dans cette zone';
container.appendChild(area);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.maxHeight = '150px';
output.style.overflow = 'auto';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

let sampleCount = 0;

// Événement de mouvement de souris
fromEvent<MouseEvent>(area, 'mousemove').pipe(
  map(event => ({
    x: event.offsetX,
    y: event.offsetY,
    timestamp: Date.now()
  })),
  sampleTime(1000) // Échantillonner chaque seconde
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>Échantillon #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    Position: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // Afficher maximum 10 entrées
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

- Même si la souris continue de bouger, seule la dernière position est échantillonnée chaque seconde.
- Si la souris n'est pas déplacée pendant une seconde, rien n'est émis pendant cette période.

## 🎯 Exemple de code pratique 2 : Tableau de bord de données temps réel

Un exemple d'échantillonnage périodique de données de capteur et d'affichage sur un tableau de bord.

```ts
import { interval } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// Création de l'UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Tableau de bord de surveillance de capteurs';
container.appendChild(title);

const dashboard = document.createElement('div');
dashboard.style.display = 'grid';
dashboard.style.gridTemplateColumns = '1fr 1fr';
dashboard.style.gap = '10px';
dashboard.style.marginTop = '10px';
container.appendChild(dashboard);

// Création des cartes de tableau de bord
function createCard(label: string, unit: string) {
  const card = document.createElement('div');
  card.style.padding = '20px';
  card.style.border = '2px solid #2196F3';
  card.style.borderRadius = '8px';
  card.style.backgroundColor = '#E3F2FD';

  const labelDiv = document.createElement('div');
  labelDiv.textContent = label;
  labelDiv.style.fontSize = '14px';
  labelDiv.style.color = '#666';
  card.appendChild(labelDiv);

  const valueDiv = document.createElement('div');
  valueDiv.style.fontSize = '32px';
  valueDiv.style.fontWeight = 'bold';
  valueDiv.style.marginTop = '10px';
  valueDiv.textContent = '--';
  card.appendChild(valueDiv);

  const unitDiv = document.createElement('div');
  unitDiv.textContent = unit;
  unitDiv.style.fontSize = '14px';
  unitDiv.style.color = '#666';
  card.appendChild(unitDiv);

  dashboard.appendChild(card);
  return valueDiv;
}

const tempValue = createCard('Température', '°C');
const humidityValue = createCard('Humidité', '%');
const pressureValue = createCard('Pression', 'hPa');
const lightValue = createCard('Luminosité', 'lux');

// Flux de données de capteur (mise à jour toutes les 100ms)
const sensorData$ = interval(100).pipe(
  map(() => ({
    temperature: (20 + Math.random() * 10).toFixed(1),
    humidity: (40 + Math.random() * 40).toFixed(1),
    pressure: (1000 + Math.random() * 30).toFixed(1),
    light: Math.floor(Math.random() * 1000)
  }))
);

// Échantillonner toutes les 2 secondes et mettre à jour le tableau de bord
sensorData$.pipe(
  sampleTime(2000)
).subscribe(data => {
  tempValue.textContent = data.temperature;
  humidityValue.textContent = data.humidity;
  pressureValue.textContent = data.pressure;
  lightValue.textContent = data.light.toString();

  // Effet d'animation
  [tempValue, humidityValue, pressureValue, lightValue].forEach(elem => {
    elem.style.color = '#2196F3';
    setTimeout(() => {
      elem.style.color = 'black';
    }, 500);
  });
});
```

- Les données de capteur sont mises à jour toutes les 100ms, mais le tableau de bord est mis à jour avec les valeurs échantillonnées toutes les 2 secondes.
- En affichant des flux de données à haute fréquence à des intervalles appropriés, les performances peuvent être optimisées.

## 🆚 Comparaison avec des opérateurs similaires

### sampleTime vs throttleTime vs auditTime

```ts
import { interval } from 'rxjs';
import { sampleTime, throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, ...

// sampleTime: échantillonne la dernière valeur chaque seconde
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));
// Exemple de sortie: 2, 5, 8 (snapshots chaque seconde)

// throttleTime: émet la première valeur, ignore pendant 1 seconde
source$.pipe(
  throttleTime(1000)
).subscribe(val => console.log('throttleTime:', val));
// Exemple de sortie: 0, 3, 6, 9 (première valeur de chaque période)

// auditTime: émet la dernière valeur 1 seconde après la première
source$.pipe(
  auditTime(1000)
).subscribe(val => console.log('auditTime:', val));
// Exemple de sortie: 2, 5, 8 (dernière valeur de chaque période)
```

| Opérateur | Moment de déclenchement | Valeur émise | Cas d'utilisation |
|:---|:---|:---|:---|
| `sampleTime(1000)` | **Timing périodique chaque seconde** | Dernière valeur à ce moment | Snapshots périodiques |
| `throttleTime(1000)` | Ignore pendant 1 seconde après réception | Première valeur au début de la période | Réduction d'événements |
| `auditTime(1000)` | 1 seconde après réception de la valeur | Dernière valeur de la période | Dernier état de la période |

**Différences visuelles** :

```
Entrée: --|1|2|3|---|4|5|6|---|7|8|9|
        0s  1s      2s      3s

sampleTime(1s):  -------|3|-------|6|-------|9|
                 (échantillonnage périodique)

throttleTime(1s): |1|--------------|4|--------------|7|
                  (passe le premier, ignore la période)

auditTime(1s):    -------|3|-------|6|-------|9|
                  (dernière valeur à la fin de la période)
```

## ⚠️ Points d'attention

### 1. Quand il n'y a pas de valeur pendant la période d'échantillonnage

Si aucune nouvelle valeur n'existe au moment de l'échantillonnage, rien n'est émis.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Échantillon récupéré');
});
// Si aucun clic pendant 2 secondes, rien n'est émis
```

### 2. Attente jusqu'au premier timing d'échantillonnage

`sampleTime` n'émet rien jusqu'à ce que le temps spécifié soit écoulé.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

interval(100).pipe(
  sampleTime(1000)
).subscribe(console.log);
// La première valeur est émise après 1 seconde
```

### 3. Timing de complétion

Même si la source termine, la complétion n'est pas propagée jusqu'au prochain timing d'échantillonnage.

```ts
import { of } from 'rxjs';
import { sampleTime, delay } from 'rxjs';

of(1, 2, 3).pipe(
  delay(100),
  sampleTime(1000)
).subscribe({
  next: console.log,
  complete: () => console.log('Terminé')
});
// Après 1 seconde: 3
// Après 1 seconde: Terminé
```

### 4. Utilisation mémoire

Ne conserve qu'une seule dernière valeur en interne, donc l'efficacité mémoire est bonne.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

// Flux haute fréquence (toutes les 10ms)
interval(10).pipe(
  sampleTime(1000) // Échantillonne chaque seconde
).subscribe(console.log);
// Seule la dernière valeur est conservée en mémoire
```

## 💡 Différence avec sample

`sample` utilise un autre Observable comme déclencheur, tandis que `sampleTime` utilise un intervalle de temps fixe.

```ts
import { interval, fromEvent } from 'rxjs';
import { sample, sampleTime } from 'rxjs';

const source$ = interval(100);

// sampleTime: intervalle de temps fixe (chaque seconde)
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));

// sample: utilise un autre Observable comme déclencheur
const clicks$ = fromEvent(document, 'click');
source$.pipe(
  sample(clicks$)
).subscribe(val => console.log('sample:', val));
// Émet la dernière valeur à chaque clic
```

| Opérateur | Déclencheur | Cas d'utilisation |
|:---|:---|:---|
| `sampleTime(ms)` | Intervalle de temps fixe | Échantillonnage périodique |
| `sample(notifier$)` | Autre Observable | Échantillonnage à timing dynamique |

## 📚 Opérateurs associés

- **[sample](https://rxjs.dev/api/operators/sample)** - Échantillonner avec un autre Observable comme déclencheur (documentation officielle)
- **[throttleTime](./throttleTime)** - Récupérer la première valeur au début de la période
- **[auditTime](./auditTime)** - Récupérer la dernière valeur à la fin de la période
- **[debounceTime](./debounceTime)** - Émettre la valeur après silence

## Résumé

L'opérateur `sampleTime` échantillonne périodiquement la dernière valeur à intervalles réguliers.

- ✅ Idéal pour les snapshots périodiques
- ✅ Efficace pour réduire les flux haute fréquence
- ✅ Bonne efficacité mémoire (ne conserve qu'une valeur)
- ✅ Idéal pour les tableaux de bord et la surveillance
- ⚠️ N'émet rien s'il n'y a pas de valeur pendant la période d'échantillonnage
- ⚠️ Temps d'attente jusqu'au premier échantillon
- ⚠️ La complétion est propagée au prochain timing d'échantillonnage
