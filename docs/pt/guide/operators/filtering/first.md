---
description: O operador first recupera apenas o primeiro valor do stream, ou o primeiro valor que satisfaz a condição especificada, e então completa o stream. Isso é útil quando você quer processar apenas o primeiro evento alcançado ou recuperar dados iniciais.
titleTemplate: ':title'
---

# first - Obter Primeiro Valor

O operador `first` recupera apenas o **primeiro valor** ou **primeiro valor que satisfaz uma condição** de um stream e completa o stream.


## 🔰 Sintaxe Básica e Uso

```ts
import { from } from 'rxjs';
import { first } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Obter apenas o primeiro valor
numbers$.pipe(
  first()
).subscribe(console.log);

// Obter apenas o primeiro valor que satisfaz a condição
numbers$.pipe(
  first(n => n > 3)
).subscribe(console.log);

// Saída:
// 1
// 4
```

- `first()` obtém o primeiro valor que flui e completa.
- Se uma condição for passada, o **primeiro valor que atende à condição** é recuperado.
- Se nenhum valor correspondente à condição existir, um erro é gerado.

[🌐 Documentação Oficial do RxJS - `first`](https://rxjs.dev/api/operators/first)


## 💡 Padrões de Uso Típicos

- Processar apenas o primeiro evento alcançado
- Detectar os primeiros dados que atendem aos critérios (por exemplo, uma pontuação de 5 ou superior)
- Adotar apenas os primeiros dados que chegaram antes de um timeout ou cancelamento


## 🧠 Exemplo de Código Prático (com UI)

Processar **apenas o primeiro clique** mesmo se o botão for clicado várias vezes.

```ts
import { fromEvent } from 'rxjs';
import { first } from 'rxjs';

const title = document.createElement('div');
title.innerHTML = '<h3>Exemplo Prático de first:</h3>';
document.body.appendChild(title);

// Criar botão
const button = document.createElement('button');
button.textContent = 'Por favor, clique (responde apenas na primeira vez)';
document.body.appendChild(button);

// Criar área de saída
let count = 0;
const output = document.createElement('div');
document.body.appendChild(output);
// Stream de clique do botão
fromEvent(button, 'click')
  .pipe(first())
  .subscribe(() => {
    const message = document.createElement('div');
    count++;
    message.textContent = `Primeiro clique detectado! ${count}`;
    output.appendChild(message);
  });
```

- Apenas o primeiro evento de clique é recebido, e eventos subsequentes são ignorados.
- O stream será automaticamente `complete` após o primeiro clique.
