---
description: "L'opérateur timestamp attache un horodatage à chaque valeur, enregistrant le moment où les valeurs ont été émises pour la mesure des performances et le débogage."
---

# timestamp - Attacher des horodatages

L'opérateur `timestamp` **attache un horodatage à chaque valeur** dans un flux. En enregistrant l'heure exacte à laquelle les valeurs ont été émises, il peut être utilisé pour la mesure des performances, le débogage et l'analyse chronologique des événements.

## 🔰 Syntaxe et comportement de base

Convertit chaque valeur en un objet avec un horodatage.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

interval(1000)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(console.log);

// Sortie :
// { value: 0, timestamp: 1640000000000 }
// { value: 1, timestamp: 1640000001000 }
// { value: 2, timestamp: 1640000002000 }
```

L'objet retourné a la structure suivante :
- `value` : La valeur originale
- `timestamp` : L'horodatage (temps Unix en millisecondes)

[🌐 Documentation officielle RxJS - timestamp](https://rxjs.dev/api/index/function/timestamp)

## 💡 Cas d'utilisation typiques

- **Mesure des performances** : Mesure du temps de traitement
- **Analyse du timing des événements** : Mesure des intervalles entre les actions utilisateur
- **Débogage et journalisation** : Enregistrement du moment d'émission des valeurs
- **Enregistrement de données temporelles** : Sauvegarde de données de capteurs avec horodatage

## 🧪 Exemple de code pratique 1 : Mesure des intervalles de clic

Exemple mesurant les intervalles entre les clics de l'utilisateur.

```ts
import { fromEvent } from 'rxjs';
import { timestamp, pairwise, map } from 'rxjs';

// Création de l'interface utilisateur
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'timestamp - Mesure des intervalles de clic';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Cliquez ici';
button.style.marginBottom = '10px';
button.style.padding = '10px 20px';
button.style.fontSize = '16px';
container.appendChild(button);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '250px';
output.style.overflow = 'auto';
container.appendChild(output);

let clickCount = 0;

function addLog(message: string, color: string = '#e3f2fd') {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;
  logItem.textContent = message;
  output.insertBefore(logItem, output.firstChild);
}

fromEvent(button, 'click')
  .pipe(
    timestamp(),
    pairwise(),
    map(([prev, curr]) => {
      const interval = curr.timestamp - prev.timestamp;
      return {
        clickNumber: clickCount + 1,
        interval: interval,
        timestamp: new Date(curr.timestamp).toLocaleTimeString('fr-FR')
      };
    })
  )
  .subscribe(data => {
    clickCount++;
    const color = data.interval < 500 ? '#ffcdd2' :
                  data.interval < 1000 ? '#fff9c4' : '#c8e6c9';

    const speed = data.interval < 500 ? 'Clic rapide !' :
                  data.interval < 1000 ? 'Normal' : 'Lent';

    addLog(
      `${data.clickNumber}e : ${data.interval}ms d'intervalle [${speed}] (${data.timestamp})`,
      color
    );
  });

addLog('Cliquez sur le bouton (mesure à partir du 2e clic)', '#e3f2fd');
```

- Mesure précise des intervalles entre les clics
- Affichage coloré selon la vitesse
- Enregistrement de l'heure d'occurrence avec l'horodatage

## 🧪 Exemple de code pratique 2 : Mesure du temps de traitement

Exemple mesurant le temps pris par chaque traitement.

```ts
import { interval } from 'rxjs';
import { timestamp, map, take } from 'rxjs';

// Création de l'interface utilisateur
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'timestamp - Mesure du temps de traitement';
container2.appendChild(title2);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '3px';
  logItem.style.fontSize = '12px';
  logItem.style.fontFamily = 'monospace';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

addLog2('Début du traitement...');

interval(500)
  .pipe(
    take(5),
    timestamp(),
    map(data => {
      const start = data.timestamp;

      // Simulation d'un traitement lourd (temps de traitement aléatoire)
      const iterations = Math.floor(Math.random() * 5000000) + 1000000;
      let sum = 0;
      for (let i = 0; i < iterations; i++) {
        sum += i;
      }

      const end = Date.now();
      const duration = end - start;

      return {
        value: data.value,
        startTime: new Date(start).toLocaleTimeString('fr-FR', { hour12: false }) +
                   '.' + (start % 1000).toString().padStart(3, '0'),
        duration: duration
      };
    })
  )
  .subscribe({
    next: result => {
      addLog2(
        `Valeur ${result.value} : début=${result.startTime}, temps de traitement=${result.duration}ms`
      );
    },
    complete: () => {
      addLog2('--- Tous les traitements sont terminés ---');
    }
  });
```

- Enregistrement de l'heure de début de traitement pour chaque valeur
- Mesure du temps pris pour le traitement
- Utilisation pour l'analyse des performances

## Utilisation des horodatages

```ts
import { of } from 'rxjs';
import { timestamp, map } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    timestamp(),
    map(data => {
      // Traitement avec l'horodatage
      const date = new Date(data.timestamp);
      return {
        value: data.value,
        time: date.toISOString(),
        unixTime: data.timestamp
      };
    })
  )
  .subscribe(console.log);
// Sortie :
// { value: 'A', time: '2024-01-01T00:00:00.000Z', unixTime: 1704067200000 }
// ...
```

## ⚠️ Notes importantes

### 1. Précision de l'horodatage

Utilise `Date.now()` de JavaScript, donc la précision est en millisecondes.

```ts
import { interval } from 'rxjs';
import { timestamp, take } from 'rxjs';

// Événements à haute fréquence (intervalle de 1ms)
interval(1)
  .pipe(
    take(3),
    timestamp()
  )
  .subscribe(data => {
    console.log(`Valeur : ${data.value}, Horodatage : ${data.timestamp}`);
  });
// Peut avoir le même horodatage
```

Pour une plus grande précision, envisagez d'utiliser `performance.now()`.

### 2. L'horodatage est au moment de l'émission

L'horodatage correspond au moment de l'émission de la valeur, pas au moment de sa génération.

```ts
import { of, delay, timestamp } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delay(1000),      // Délai de 1 seconde
    timestamp()       // Horodatage après le délai
  )
  .subscribe(console.log);
```

### 3. Changement de structure de l'objet

Lorsque vous utilisez `timestamp`, les valeurs sont enveloppées dans un objet.

```ts
import { of, timestamp, map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    timestamp(),
    map(data => data.value * 2)  // Accès à la valeur originale via .value
  )
  .subscribe(console.log);
// Sortie : 2, 4, 6
```

## 📚 Opérateurs associés

- **[tap](./tap)** - Exécution d'effets secondaires (pour le débogage)
- **[delay](./delay)** - Délai fixe
- **[timeout](./timeout)** - Contrôle du délai d'expiration

## ✅ Résumé

L'opérateur `timestamp` attache un horodatage à chaque valeur.

- ✅ Enregistrement précis de l'heure d'émission de chaque valeur
- ✅ Efficace pour la mesure des performances
- ✅ Analyse des intervalles entre événements possible
- ✅ Utile pour le débogage et la journalisation
- ⚠️ Précision en millisecondes
- ⚠️ Les valeurs sont enveloppées dans un objet
- ⚠️ L'horodatage correspond au moment de l'émission
