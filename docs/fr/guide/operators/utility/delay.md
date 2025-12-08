---
description: "L'opérateur delay retarde le moment de l'émission de chaque valeur d'un Observable d'une durée spécifiée. Efficace pour la mise en scène de l'interface utilisateur, la limitation du débit, le contrôle du traitement asynchrone et la simulation des retards pendant les tests. La différence avec delayWhen et l'implémentation type-safe en TypeScript sont expliquées avec des exemples de code pratiques."
---

# delay - Retarder les valeurs

L'opérateur `delay` est utilisé pour retarder la publication de chaque valeur dans un flux pour un temps spécifié.
Ceci est utile pour mettre en scène des animations et ajuster le timing de l'affichage du feedback à l'utilisateur.


## 🔰 Syntaxe et comportement de base

Configuration minimale pour émettre une valeur après un certain laps de temps.

```ts
import { of } from 'rxjs';
import { delay } from 'rxjs';

of('Hello')
  .pipe(
    delay(1000) // Émettre la valeur après 1 seconde
  )
  .subscribe(console.log);
// Sortie :
// Hello
```

Dans cet exemple, la valeur créée par `of('Hello')` est reçue par `subscribe()` après un délai d'une seconde.

[🌐 Documentation officielle RxJS - delay](https://rxjs.dev/api/index/function/delay)

## 💡 Cas d'utilisation typiques

Ceci est un exemple d'utilisation du délai pour ajuster le timing d'émission dans une situation où plusieurs valeurs sont émises.

```ts
import { of } from 'rxjs';
import { delay, concatMap } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    concatMap(
      (val, index) => of(val).pipe(delay(1000 * index)) // A immédiatement, B après 1 seconde, C après 2 secondes
    )
  )
  .subscribe(console.log);
// Sortie :
// A
// B
```

De cette façon, en combinaison avec `concatMap`, il est également possible de définir un délai distinct pour chaque valeur.


## 🧪 Exemple de code pratique (avec interface utilisateur)

```ts
import { of } from 'rxjs';
import { delay, tap } from 'rxjs';

// Zone d'affichage de la sortie
const delayOutput = document.createElement('div');
delayOutput.innerHTML = '<h3>Exemple de delay :</h3>';
document.body.appendChild(delayOutput);

// Fonction d'affichage de l'heure actuelle
function addTimeLog(message: string) {
  const now = new Date();
  const time =
    now.toLocaleTimeString('fr-FR', { hour12: false }) +
    '.' +
    now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `${time}: ${message}`;
  delayOutput.appendChild(logItem);
}

// Enregistrement de l'heure de début
addTimeLog('Démarrage');

// Séquence de valeurs
of('A', 'B', 'C')
  .pipe(
    tap((val) => addTimeLog(`Valeur ${val} avant émission`)),
    delay(1000), // Délai d'une seconde
    tap((val) => addTimeLog(`Valeur ${val} émise après 1 seconde`))
  )
  .subscribe();
```


## ✅ Résumé

- `delay` est un opérateur pour **contrôler le timing de la sortie d'un Observable**
- Il peut être utilisé pour appliquer un délai constant ou combiné avec `concatMap` pour **contrôler le délai valeur par valeur**
- Utile pour les **ajustements asynchrones** afin d'améliorer l'UX, comme la sortie vers l'interface utilisateur ou la mise en scène du timer
