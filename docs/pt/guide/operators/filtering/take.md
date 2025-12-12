---
description: O operador take recupera apenas o número especificado de primeiros valores do stream Observable e completa automaticamente o stream, ignorando valores subsequentes. Isso é útil quando você quer recuperar apenas os primeiros dados.
titleTemplate: ':title | RxJS'
---

# take - Recuperar Apenas o Número Especificado de Primeiros Valores

O operador `take` recupera apenas o **número especificado de primeiros** valores do stream e ignora valores subsequentes.
Após a conclusão, o stream automaticamente `completa`.

## 🔰 Sintaxe Básica e Uso

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  take(3)
).subscribe(console.log);
// Saída: 0, 1, 2
```

- Inscreve-se apenas nos primeiros 3 valores.
- Após recuperar 3 valores, o Observable automaticamente `completa`.

[🌐 Documentação Oficial do RxJS - `take`](https://rxjs.dev/api/operators/take)

## 💡 Padrões de Uso Típicos

- Exibir ou registrar apenas os primeiros itens na UI
- Inscrição temporária para recuperar apenas a primeira resposta
- Recuperação limitada de dados de teste ou demonstração

## 🧠 Exemplo de Código Prático (com UI)

Recupera e exibe apenas os primeiros 5 valores de números emitidos a cada segundo.

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>Exemplo Prático de take:</h3>';
document.body.appendChild(output);

// Emitir valores a cada segundo
const source$ = interval(1000);

// Pegar apenas os primeiros 5 valores
source$.pipe(take(5)).subscribe({
  next: (value) => {
    const item = document.createElement('div');
    item.textContent = `Valor: ${value}`;
    output.appendChild(item);
  },
  complete: () => {
    const complete = document.createElement('div');
    complete.textContent = 'Completado';
    complete.style.fontWeight = 'bold';
    output.appendChild(complete);
  },
});

```

- Os primeiros 5 valores (`0`, `1`, `2`, `3`, `4`) são exibidos em ordem,
- Então a mensagem "Completado" é exibida.
