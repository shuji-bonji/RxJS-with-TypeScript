---
description: "L'opérateur sampleTime est un opérateur de filtrage RxJS qui échantillonne périodiquement les dernières valeurs du flux à des intervalles de temps spécifiés. Il est idéal pour prendre des instantanés périodiques."
---

# sampleTime - obtient périodiquement la dernière valeur

L'opérateur `sampleTime` **échantillonne** périodiquement la dernière valeur de l'observable source à **intervalles de temps spécifiés** et la restitue.
Comme un instantané périodique, il récupère la valeur la plus récente à ce moment-là.

## 🔰 Syntaxe de base et utilisation

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Échantillons seconde par seconde');
});
```

**Flux d'opérations** :.
1. le timer se déclenche périodiquement toutes les 2 secondes
2. sortie s'il y a un clic récent à ce moment-là
3. s'il n'y a pas de valeur pendant la période d'échantillonnage, pas de sortie

> [!WARNING] 本番コードでの注意

> L'exemple ci-dessus omet la désinscription de `fromEvent` pour simplifier l'explication. Dans le code réel, utilisez `takeUntil(destroy$)`, `take(N)` ou `Subscription.unsubscribe()` pour gérer explicitement le cycle de vie. Plus d'informations : [Surmonter les difficultés : gestion du cycle de vie](/fr/guide/overcoming-difficulties/lifecycle-management.md)

[🌐 Official RxJS documentation - `sampleTime`](https://rxjs.dev/api/operators/sampleTime)

## 💡 Modèles d'utilisation typiques

- **Acquisition récurrente de données de capteurs** : informations de température et de localisation mises à jour toutes les secondes.
- Tableau de bord en temps réel : mises à jour régulières de l'état de la situation.
- Contrôle des performances** : collecte de mesures à intervalles réguliers.
- Traitement des images de jeu** : échantillonnage périodique pour le contrôle du FPS

## 🧠 Exemple de code pratique 1 : Échantillonnage périodique de la position de la souris

Voici un exemple d'échantillonnage de la position de la souris toutes les secondes.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UICréation
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Échantillonnage de la position de la souris (1(toutes les secondes)';
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
area.textContent = 'Déplacement de la souris dans cette zone';
container.appendChild(area);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.maxHeight = '150px';
output.style.overflow = 'auto';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
container.appendChild(output);

let sampleCount = 0;

// Événement de déplacement de la souris
fromEvent<MouseEvent>(area, 'mousemove').pipe(
  map(event => ({
    x: event.offsetX,
    y: event.offsetY,
    timestamp: Date.now()
  })),
  sampleTime(1000) // 1Échantillonnage toutes les secondes
).subscribe(pos => {
  sampleCount++;
  const log = document.createElement('div');
  log.style.padding = '5px';
  log.style.borderBottom = '1px solid #eee';
  log.innerHTML = `
    <strong>Échantillons #${sampleCount}</strong>
    [${new Date(pos.timestamp).toLocaleTimeString()}]
    Position: (${pos.x}, ${pos.y})
  `;
  output.insertBefore(log, output.firstChild);

  // Max.10Affichage de jusqu'à
  while (output.children.length > 10) {
    output.removeChild(output.lastChild!);
  }
});
```

- Si la souris est déplacée en permanence, seule la dernière position actuelle est échantillonnée toutes les secondes.
- Si la souris n'est pas déplacée pendant une seconde, rien n'est émis pendant cette période.

## 🎯 Exemple de code pratique 2 : tableau de bord en temps réel

Cet exemple montre comment les données d'un capteur peuvent être échantillonnées périodiquement et affichées sur un tableau de bord.

```ts
import { interval } from 'rxjs';
import { sampleTime, map } from 'rxjs';

// UICréation
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Tableau de bord de surveillance des capteurs';
container.appendChild(title);

const dashboard = document.createElement('div');
dashboard.style.display = 'grid';
dashboard.style.gridTemplateColumns = '1fr 1fr';
dashboard.style.gap = '10px';
dashboard.style.marginTop = '10px';
container.appendChild(dashboard);

// Création d'une carte de tableau de bord
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
const pressureValue = createCard('Pression barométrique', 'hPa');
const lightValue = createCard('L'éclairement', 'lux');

// Flux de données des capteurs (100msMise à jour toutes les)
const sensorData$ = interval(100).pipe(
  map(() => ({
    temperature: (20 + Math.random() * 10).toFixed(1),
    humidity: (40 + Math.random() * 40).toFixed(1),
    pressure: (1000 + Math.random() * 30).toFixed(1),
    light: Math.floor(Math.random() * 1000)
  }))
);

// 2Tableau de bord échantillonné et mis à jour toutes les secondes
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

- Les données du capteur sont mises à jour toutes les 100 ms, tandis que le tableau de bord est mis à jour avec des valeurs échantillonnées toutes les 2 secondes.
- Les performances peuvent être optimisées en affichant des flux de données à haute fréquence à des intervalles appropriés.

## 🆚 Comparaison avec des opérateurs similaires

### sampleTime vs throttleTime vs auditTime

```ts
import { interval } from 'rxjs';
import { sampleTime, throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, ...

// sampleTime: 1Échantillonnage de la dernière valeur à ce moment-là toutes les secondes
source$.pipe(
  sampleTime(1000)
).subscribe(val => console.log('sampleTime:', val));
// Exemples de sorties: 2, 5, 8(1Instantané toutes les secondes)

// throttleTime: Après la sortie de la première valeur,1Ignoré pendant 2 secondes après la sortie de la première valeur
source$.pipe(
  throttleTime(1000)
).subscribe(val => console.log('throttleTime:', val));
// Exemples de sorties: 0, 3, 6, 9(première valeur de chaque période)

// auditTime: Sortie de la dernière valeur de la période1secondes après la première valeur, la dernière valeur de la période est émise
source$.pipe(
  auditTime(1000)
).subscribe(val => console.log('auditTime:', val));
// Exemples de sorties: 2, 5, 8(dernière valeur de chaque période)
```

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Échantillons seconde par seconde');
});
```1___

**différences visuelles** :.

```
Entrée: --|1|2|3|---|4|5|6|---|7|8|9|
      0s  1s      2s      3s

sampleTime(1s):  -------|3|-------|6|-------|9|
                 (Échantillonnage périodique)

throttleTime(1s): |1|--------------|4|--------------|7|
                  (Ignoré pendant la période jusqu'au début)

auditTime(1s):    -------|3|-------|6|-------|9|
                  (Dernière valeur à la fin de la période)
```

## ⚠️ Notes.

### 1. pas de valeur pendant la période d'échantillonnage

S'il n'y a pas de nouvelles valeurs pendant la période d'échantillonnage, aucune sortie n'est produite.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('Échantillons prélevés');
});
// 2Pendant les secondes1Pas de sortie si aucun clic n'est effectué
```

### Attendre jusqu'au premier moment d'échantillonnage

L'option `sampleTime` ne produira rien tant que le temps spécifié ne se sera pas écoulé.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

interval(100).pipe(
  sampleTime(1000)
).subscribe(console.log);
// La première valeur est1secondes après la sortie de la première valeur
```

### 3. completionTime

Lorsqu'une source s'achève, l'achèvement n'est pas propagé jusqu'à la prochaine synchronisation de l'échantillon.

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
// 1Quelques secondes plus tard: 3
// 1Quelques secondes plus tard: Terminé
```

### 4. utilisation de la mémoire

L'efficacité de la mémoire est bonne car seule la dernière valeur est conservée en interne.

```ts
import { interval } from 'rxjs';
import { sampleTime } from 'rxjs';

// Flux à haute fréquence (10mspar seconde)
interval(10).pipe(
  sampleTime(1000) // 1Échantillonnage toutes les secondes
).subscribe(console.log);
// La mémoire ne retient que la dernière valeur1Seules les deux valeurs les plus récentes sont conservées en mémoire
```

## 💡 Différences avec l'échantillon

`sample` utilise un autre Observable comme déclencheur, alors que `sampleTime` utilise un intervalle de temps fixe.

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Échantillons seconde par seconde');
});
```0___

```ts
import { fromEvent } from 'rxjs';
import { sampleTime } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  sampleTime(2000)
).subscribe(() => {
  console.log('2Échantillons seconde par seconde');
});
```2___

## 📚 Opérateurs apparentés.

- **[sample](https://rxjs.dev/api/operators/sample)** - Échantillonnage d'un autre Observable comme déclencheur (documentation officielle).
- **[throttleTime](. /throttleTime)** - Obtenir la première valeur au début de la période.
- **[auditTime](. /auditTime)** - Obtient la dernière valeur à la fin de la période.
- **[debounceTime](. /debounceTime)** - émet une valeur après la quiescence

## Résumé.

L'opérateur `sampleTime` échantillonne périodiquement la dernière valeur à l'intervalle de temps spécifié.

- ✅ Idéal pour prendre des instantanés périodiques
- ✅ Utile pour éclaircir les flux à haute fréquence
- Efficace en termes de mémoire (seule la dernière valeur est conservée)
- Idéal pour les tableaux de bord et la surveillance
- ⚠️ Si aucune valeur n'est disponible pendant la période d'échantillonnage, rien n'est produit.
- ⚠️ Il y a une période d'attente jusqu'au premier échantillon.
- ⚠️ L'achèvement est propagé au moment de l'échantillonnage suivant.
